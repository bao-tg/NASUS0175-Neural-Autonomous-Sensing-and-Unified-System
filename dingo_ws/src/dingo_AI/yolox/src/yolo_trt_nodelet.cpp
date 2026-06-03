#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <memory>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

namespace yolox {

class TrtLogger : public nvinfer1::ILogger {
public:
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            ROS_WARN_STREAM("[TensorRT] " << msg);
        }
    }
};

struct TrtDestroy {
    template <typename T>
    void operator()(T* obj) const {
        if (obj) obj->destroy();
    }
};

template <typename T>
using TrtUniquePtr = std::unique_ptr<T, TrtDestroy>;

struct Detection {
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
    float score = 0.0f;
    int cls = -1;
};

struct Track {
    int id = 0;
    Detection det;
    int missed = 0;
};

static std::vector<char> readFile(const std::string& path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        throw std::runtime_error("failed to open engine: " + path);
    }
    const std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> data(static_cast<size_t>(size));
    if (!file.read(data.data(), size)) {
        throw std::runtime_error("failed to read engine: " + path);
    }
    return data;
}

static size_t volume(const nvinfer1::Dims& dims) {
    size_t v = 1;
    for (int i = 0; i < dims.nbDims; ++i) {
        v *= static_cast<size_t>(dims.d[i]);
    }
    return v;
}

static std::string dimsToString(const nvinfer1::Dims& dims) {
    std::ostringstream oss;
    oss << "[";
    for (int i = 0; i < dims.nbDims; ++i) {
        if (i) oss << "x";
        oss << dims.d[i];
    }
    oss << "]";
    return oss.str();
}

static float iou(const Detection& a, const Detection& b) {
    const float ix1 = std::max(a.x1, b.x1);
    const float iy1 = std::max(a.y1, b.y1);
    const float ix2 = std::min(a.x2, b.x2);
    const float iy2 = std::min(a.y2, b.y2);
    const float iw = std::max(0.0f, ix2 - ix1);
    const float ih = std::max(0.0f, iy2 - iy1);
    const float inter = iw * ih;
    const float areaA = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    const float areaB = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
    const float uni = areaA + areaB - inter;
    return uni > 0.0f ? inter / uni : 0.0f;
}

static std::vector<Detection> nms(std::vector<Detection> dets, float threshold) {
    std::sort(dets.begin(), dets.end(), [](const Detection& a, const Detection& b) {
        return a.score > b.score;
    });
    std::vector<Detection> kept;
    std::vector<bool> removed(dets.size(), false);
    for (size_t i = 0; i < dets.size(); ++i) {
        if (removed[i]) continue;
        kept.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (!removed[j] && dets[i].cls == dets[j].cls && iou(dets[i], dets[j]) > threshold) {
                removed[j] = true;
            }
        }
    }
    return kept;
}

static cv::Mat imageMsgToBgr(const sensor_msgs::Image& msg) {
    if (msg.encoding != "bgr8" && msg.encoding != "rgb8" && msg.encoding != "mono8") {
        throw std::runtime_error("unsupported image encoding: " + msg.encoding);
    }
    const int channels = msg.encoding == "mono8" ? 1 : 3;
    cv::Mat raw(static_cast<int>(msg.height), static_cast<int>(msg.width), channels == 1 ? CV_8UC1 : CV_8UC3, const_cast<uint8_t*>(msg.data.data()), msg.step);
    cv::Mat compact = raw.clone();
    if (msg.encoding == "rgb8") {
        cv::cvtColor(compact, compact, cv::COLOR_RGB2BGR);
    } else if (msg.encoding == "mono8") {
        cv::cvtColor(compact, compact, cv::COLOR_GRAY2BGR);
    }
    return compact;
}

class YoloTrtNodelet : public nodelet::Nodelet {
public:
    ~YoloTrtNodelet() override {
        if (stream_) cudaStreamDestroy(stream_);
        for (void* ptr : deviceBindings_) {
            if (ptr) cudaFree(ptr);
        }
    }

private:
    void onInit() override {
        try {
            nh_ = getMTNodeHandle();
            privateNh_ = getMTPrivateNodeHandle();
            privateNh_.param<std::string>("image_topic", imageTopic_, "/camera/image_raw");
        privateNh_.param<std::string>("model_path", modelPath_, "/dingo_ws/src/dingo_AI/yolox/models/yolov8n.engine");
        privateNh_.param<std::string>("tracking_backend", trackingBackend_, "iou");
        privateNh_.param<double>("conf", confThreshold_, 0.25);
        privateNh_.param<double>("iou", nmsThreshold_, 0.7);
        privateNh_.param<double>("iou_tracker_threshold", trackerIouThreshold_, 0.3);
        privateNh_.param<int>("iou_tracker_max_missed", maxMissed_, 10);
        privateNh_.param<int>("classes", classId_, 0);
        privateNh_.param<bool>("publish_debug_image", publishDebug_, true);
        privateNh_.param<int>("max_width", maxWidth_, 640);

        if (trackingBackend_ != "iou") {
            ROS_WARN_STREAM("C++ TensorRT node currently implements IoU tracking; requested " << trackingBackend_ << " will use IoU fallback.");
        }

        loadEngine();

        bboxPub_ = privateNh_.advertise<std_msgs::Float32MultiArray>("bbox", 1);
        tracksPub_ = privateNh_.advertise<std_msgs::Float32MultiArray>("tracks", 1);
        statusPub_ = privateNh_.advertise<std_msgs::String>("status", 1);
        if (publishDebug_) {
            debugPub_ = privateNh_.advertise<sensor_msgs::Image>("debug_image", 1);
        }
        sub_ = nh_.subscribe(imageTopic_, 1, &YoloTrtNodelet::imageCallback, this, ros::TransportHints().tcpNoDelay());
        ROS_INFO_STREAM("YOLO TensorRT C++ nodelet subscribed to " << imageTopic_ << " model=" << modelPath_);
        } catch (const std::exception& exc) {
            NODELET_FATAL("failed to start yolo_trt_nodelet: %s", exc.what());
            throw;
        }
    }

    void loadEngine() {
        const auto engineData = readFile(modelPath_);
        runtime_.reset(nvinfer1::createInferRuntime(logger_));
        if (!runtime_) throw std::runtime_error("failed to create TensorRT runtime");
        engine_.reset(runtime_->deserializeCudaEngine(engineData.data(), engineData.size(), nullptr));
        if (!engine_) throw std::runtime_error("failed to deserialize TensorRT engine");
        context_.reset(engine_->createExecutionContext());
        if (!context_) throw std::runtime_error("failed to create TensorRT context");

        const int nb = engine_->getNbBindings();
        deviceBindings_.assign(nb, nullptr);
        hostOutputs_.clear();
        outputBindings_.clear();

        for (int i = 0; i < nb; ++i) {
            const auto dims = engine_->getBindingDimensions(i);
            const size_t bytes = volume(dims) * sizeof(float);
            cudaMalloc(&deviceBindings_[i], bytes);
            ROS_INFO_STREAM("binding " << i << " name=" << engine_->getBindingName(i) << " dims=" << dimsToString(dims)
                            << " bytes=" << bytes << " input=" << engine_->bindingIsInput(i));
            if (engine_->bindingIsInput(i)) {
                inputBinding_ = i;
                inputDims_ = dims;
            } else {
                outputBindings_.push_back(i);
                hostOutputs_.emplace_back(volume(dims));
            }
        }

        if (inputBinding_ < 0 || outputBindings_.empty()) {
            throw std::runtime_error("engine must have one input and at least one output");
        }
        inputC_ = inputDims_.d[1];
        inputH_ = inputDims_.d[2];
        inputW_ = inputDims_.d[3];
        if (inputC_ != 3) throw std::runtime_error("expected BCHW input with 3 channels");
        cudaStreamCreate(&stream_);
    }

    std::vector<float> preprocess(const cv::Mat& bgr) {
        srcW_ = bgr.cols;
        srcH_ = bgr.rows;
        letterboxScale_ = std::min(static_cast<float>(inputW_) / static_cast<float>(srcW_), static_cast<float>(inputH_) / static_cast<float>(srcH_));
        const int newW = static_cast<int>(std::round(srcW_ * letterboxScale_));
        const int newH = static_cast<int>(std::round(srcH_ * letterboxScale_));
        padX_ = (inputW_ - newW) / 2.0f;
        padY_ = (inputH_ - newH) / 2.0f;

        cv::Mat resized;
        cv::resize(bgr, resized, cv::Size(newW, newH));
        cv::Mat canvas(inputH_, inputW_, CV_8UC3, cv::Scalar(114, 114, 114));
        resized.copyTo(canvas(cv::Rect(static_cast<int>(padX_), static_cast<int>(padY_), newW, newH)));
        cv::cvtColor(canvas, canvas, cv::COLOR_BGR2RGB);

        std::vector<float> input(static_cast<size_t>(3 * inputH_ * inputW_));
        const int area = inputH_ * inputW_;
        for (int y = 0; y < inputH_; ++y) {
            const cv::Vec3b* row = canvas.ptr<cv::Vec3b>(y);
            for (int x = 0; x < inputW_; ++x) {
                const cv::Vec3b& px = row[x];
                const int idx = y * inputW_ + x;
                input[idx] = px[0] / 255.0f;
                input[area + idx] = px[1] / 255.0f;
                input[2 * area + idx] = px[2] / 255.0f;
            }
        }
        return input;
    }

    std::vector<Detection> infer(const cv::Mat& frame) {
        std::vector<float> input = preprocess(frame);
        cudaMemcpyAsync(deviceBindings_[inputBinding_], input.data(), input.size() * sizeof(float), cudaMemcpyHostToDevice, stream_);
        if (!context_->enqueueV2(deviceBindings_.data(), stream_, nullptr)) {
            throw std::runtime_error("TensorRT enqueueV2 failed");
        }
        for (size_t oi = 0; oi < outputBindings_.size(); ++oi) {
            const int binding = outputBindings_[oi];
            cudaMemcpyAsync(hostOutputs_[oi].data(), deviceBindings_[binding], hostOutputs_[oi].size() * sizeof(float), cudaMemcpyDeviceToHost, stream_);
        }
        cudaStreamSynchronize(stream_);
        return decodeOutput(hostOutputs_[0], engine_->getBindingDimensions(outputBindings_[0]));
    }

    std::vector<Detection> decodeOutput(const std::vector<float>& out, const nvinfer1::Dims& dims) const {
        int channels = 0;
        int anchors = 0;
        bool channelFirst = true;
        if (dims.nbDims == 3) {
            channels = dims.d[1];
            anchors = dims.d[2];
        } else if (dims.nbDims == 2) {
            channels = dims.d[0];
            anchors = dims.d[1];
        } else {
            throw std::runtime_error("unsupported output dims: " + dimsToString(dims));
        }
        if (channels < 5 && anchors >= 5) {
            channelFirst = false;
            std::swap(channels, anchors);
        }

        std::vector<Detection> dets;
        const int numClasses = channels - 4;
        for (int i = 0; i < anchors; ++i) {
            auto at = [&](int c) -> float {
                return channelFirst ? out[static_cast<size_t>(c * anchors + i)] : out[static_cast<size_t>(i * channels + c)];
            };
            int bestCls = 0;
            float bestScore = 0.0f;
            for (int c = 0; c < numClasses; ++c) {
                const float s = at(4 + c);
                if (s > bestScore) {
                    bestScore = s;
                    bestCls = c;
                }
            }
            if (bestScore < static_cast<float>(confThreshold_) || (classId_ >= 0 && bestCls != classId_)) continue;

            const float cx = at(0);
            const float cy = at(1);
            const float w = at(2);
            const float h = at(3);
            Detection det;
            det.x1 = (cx - w * 0.5f - padX_) / letterboxScale_;
            det.y1 = (cy - h * 0.5f - padY_) / letterboxScale_;
            det.x2 = (cx + w * 0.5f - padX_) / letterboxScale_;
            det.y2 = (cy + h * 0.5f - padY_) / letterboxScale_;
            det.x1 = std::max(0.0f, std::min(det.x1, static_cast<float>(srcW_ - 1)));
            det.y1 = std::max(0.0f, std::min(det.y1, static_cast<float>(srcH_ - 1)));
            det.x2 = std::max(0.0f, std::min(det.x2, static_cast<float>(srcW_ - 1)));
            det.y2 = std::max(0.0f, std::min(det.y2, static_cast<float>(srcH_ - 1)));
            det.score = bestScore;
            det.cls = bestCls;
            if (det.x2 > det.x1 && det.y2 > det.y1) dets.push_back(det);
        }
        return nms(std::move(dets), static_cast<float>(nmsThreshold_));
    }

    std::vector<Track> updateTracks(const std::vector<Detection>& detections) {
        std::vector<int> detOrder(detections.size());
        std::iota(detOrder.begin(), detOrder.end(), 0);
        std::vector<bool> detUsed(detections.size(), false);
        std::vector<bool> trackMatched(tracks_.size(), false);

        struct Pair { float score; size_t ti; size_t di; };
        std::vector<Pair> pairs;
        for (size_t ti = 0; ti < tracks_.size(); ++ti) {
            for (size_t di = 0; di < detections.size(); ++di) {
                pairs.push_back({iou(tracks_[ti].det, detections[di]), ti, di});
            }
        }
        std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.score > b.score; });

        for (const auto& pair : pairs) {
            if (pair.score < trackerIouThreshold_) break;
            if (trackMatched[pair.ti] || detUsed[pair.di]) continue;
            tracks_[pair.ti].det = detections[pair.di];
            tracks_[pair.ti].missed = 0;
            trackMatched[pair.ti] = true;
            detUsed[pair.di] = true;
        }
        for (size_t ti = 0; ti < tracks_.size(); ++ti) {
            if (!trackMatched[ti]) tracks_[ti].missed++;
        }
        tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(), [&](const Track& t) { return t.missed > maxMissed_; }), tracks_.end());
        for (size_t di = 0; di < detections.size(); ++di) {
            if (!detUsed[di]) tracks_.push_back({nextTrackId_++, detections[di], 0});
        }
        return tracks_;
    }

    void publishDebugImage(const cv::Mat& bgr, const std_msgs::Header& header) {
        sensor_msgs::Image out;
        out.header = header;
        out.height = bgr.rows;
        out.width = bgr.cols;
        out.encoding = "bgr8";
        out.is_bigendian = 0;
        out.step = static_cast<sensor_msgs::Image::_step_type>(bgr.cols * 3);
        out.data.assign(bgr.datastart, bgr.dataend);
        debugPub_.publish(out);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        cv::Mat frame;
        try {
            frame = imageMsgToBgr(*msg);
        } catch (const std::exception& exc) {
            ROS_WARN_THROTTLE(5.0, "image conversion failed: %s", exc.what());
            return;
        }
        if (maxWidth_ > 0 && frame.cols > maxWidth_) {
            const double scale = static_cast<double>(maxWidth_) / static_cast<double>(frame.cols);
            cv::resize(frame, frame, cv::Size(maxWidth_, static_cast<int>(frame.rows * scale)));
        }

        std::vector<Detection> dets;
        try {
            dets = infer(frame);
        } catch (const std::exception& exc) {
            ROS_ERROR_THROTTLE(5.0, "TensorRT inference failed: %s", exc.what());
            return;
        }
        const auto tracks = updateTracks(dets);

        std_msgs::Float32MultiArray tracksMsg;
        std_msgs::Float32MultiArray bboxMsg;
        cv::Mat debug = frame.clone();
        for (const auto& track : tracks) {
            const auto& d = track.det;
            tracksMsg.data.insert(tracksMsg.data.end(), {static_cast<float>(track.id), d.x1, d.y1, d.x2, d.y2, d.score, static_cast<float>(d.cls)});
            if (publishDebug_) {
                cv::rectangle(debug, cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)), cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)), cv::Scalar(0, 200, 255), 2);
                cv::putText(debug, "id=" + std::to_string(track.id), cv::Point(static_cast<int>(d.x1), std::max(20, static_cast<int>(d.y1) - 6)), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 200, 255), 2);
            }
        }
        if (!tracks.empty()) {
            const auto& d = tracks.front().det;
            bboxMsg.data = {1.0f, d.x1, d.y1, d.x2 - d.x1, d.y2 - d.y1};
        } else {
            bboxMsg.data = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
        }
        bboxPub_.publish(bboxMsg);
        tracksPub_.publish(tracksMsg);
        if (publishDebug_) publishDebugImage(debug, msg->header);

        frameCount_++;
        std_msgs::String status;
        status.data = "frames=" + std::to_string(frameCount_) + " detections=" + std::to_string(dets.size()) + " tracks=" + std::to_string(tracks.size()) + " backend=iou_trt_cpp";
        statusPub_.publish(status);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle privateNh_;
    ros::Subscriber sub_;
    ros::Publisher bboxPub_;
    ros::Publisher tracksPub_;
    ros::Publisher statusPub_;
    ros::Publisher debugPub_;

    std::string imageTopic_;
    std::string modelPath_;
    std::string trackingBackend_;
    double confThreshold_ = 0.25;
    double nmsThreshold_ = 0.7;
    double trackerIouThreshold_ = 0.3;
    int maxMissed_ = 10;
    int classId_ = 0;
    bool publishDebug_ = true;
    int maxWidth_ = 640;
    uint64_t frameCount_ = 0;

    TrtLogger logger_;
    TrtUniquePtr<nvinfer1::IRuntime> runtime_;
    TrtUniquePtr<nvinfer1::ICudaEngine> engine_;
    TrtUniquePtr<nvinfer1::IExecutionContext> context_;
    cudaStream_t stream_ = nullptr;
    std::vector<void*> deviceBindings_;
    std::vector<int> outputBindings_;
    std::vector<std::vector<float>> hostOutputs_;
    int inputBinding_ = -1;
    nvinfer1::Dims inputDims_{};
    int inputC_ = 0;
    int inputH_ = 0;
    int inputW_ = 0;

    int srcW_ = 0;
    int srcH_ = 0;
    float letterboxScale_ = 1.0f;
    float padX_ = 0.0f;
    float padY_ = 0.0f;

    std::vector<Track> tracks_;
    int nextTrackId_ = 1;
};
}  // namespace yolox

PLUGINLIB_EXPORT_CLASS(yolox::YoloTrtNodelet, nodelet::Nodelet)
