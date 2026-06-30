#!/usr/bin/env python3

import base64
import json
import mimetypes
import os
import tempfile
import urllib.error
import urllib.request

import rospy


OPENAI_API_BASE = os.environ.get("OPENAI_API_BASE", "https://api.openai.com/v1")


def get_api_key():
    api_key = os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError("OPENAI_API_KEY not found in the environment")
    return api_key


def openai_json_request(path, payload, timeout=60):
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(
        OPENAI_API_BASE + path,
        data=data,
        headers={
            "Authorization": "Bearer " + get_api_key(),
            "Content-Type": "application/json",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace")
        raise RuntimeError("OpenAI HTTP %s: %s" % (exc.code, body))


def chat_completion(messages, model, max_tokens=300, temperature=0.0):
    try:
        import openai

        openai.api_key = get_api_key()
        response = openai.ChatCompletion.create(
            model=model,
            messages=messages,
            max_tokens=max_tokens,
            temperature=temperature,
        )
        return response["choices"][0]["message"]["content"].strip()
    except Exception as exc:
        rospy.logwarn("OpenAI SDK chat call failed, falling back to HTTP: %s", exc)

    response = openai_json_request(
        "/chat/completions",
        {
            "model": model,
            "messages": messages,
            "max_tokens": max_tokens,
            "temperature": temperature,
        },
    )
    return response["choices"][0]["message"]["content"].strip()


def transcribe_audio(audio_path, model="gpt-4o-transcribe", language=None, prompt=None, timeout=90):
    if model == "whisper-1":
        try:
            import openai

            openai.api_key = get_api_key()
            with open(audio_path, "rb") as audio_file:
                kwargs = {"model": model, "file": audio_file}
                if language:
                    kwargs["language"] = language
                if prompt:
                    kwargs["prompt"] = prompt
                response = openai.Audio.transcribe(**kwargs)
            return response.get("text", "").strip()
        except Exception as exc:
            rospy.logwarn("OpenAI SDK transcription failed, falling back to HTTP: %s", exc)

    boundary = "----dingo-openai-boundary"
    fields = [("model", model)]
    if language:
        fields.append(("language", language))
    if prompt:
        fields.append(("prompt", prompt))

    body = bytearray()
    for name, value in fields:
        body.extend(("--%s\r\n" % boundary).encode("utf-8"))
        body.extend(('Content-Disposition: form-data; name="%s"\r\n\r\n' % name).encode("utf-8"))
        body.extend(str(value).encode("utf-8"))
        body.extend(b"\r\n")

    filename = os.path.basename(audio_path)
    content_type = mimetypes.guess_type(filename)[0] or "audio/wav"
    with open(audio_path, "rb") as audio_file:
        audio_data = audio_file.read()
    body.extend(("--%s\r\n" % boundary).encode("utf-8"))
    file_header = (
        'Content-Disposition: form-data; name="file"; filename="%s"\r\n'
        "Content-Type: %s\r\n\r\n"
    ) % (filename, content_type)
    body.extend(file_header.encode("utf-8"))
    body.extend(audio_data)
    body.extend(b"\r\n")
    body.extend(("--%s--\r\n" % boundary).encode("utf-8"))

    req = urllib.request.Request(
        OPENAI_API_BASE + "/audio/transcriptions",
        data=bytes(body),
        headers={
            "Authorization": "Bearer " + get_api_key(),
            "Content-Type": "multipart/form-data; boundary=" + boundary,
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            response = json.loads(resp.read().decode("utf-8"))
            return response.get("text", "").strip()
    except urllib.error.HTTPError as exc:
        body_text = exc.read().decode("utf-8", "replace")
        raise RuntimeError("OpenAI transcription HTTP %s: %s" % (exc.code, body_text))


def synthesize_speech(text, voice="alloy", model="tts-1", response_format="wav", timeout=90):
    payload = {
        "model": model,
        "voice": voice,
        "input": text,
        "response_format": response_format,
    }
    req = urllib.request.Request(
        OPENAI_API_BASE + "/audio/speech",
        data=json.dumps(payload).encode("utf-8"),
        headers={
            "Authorization": "Bearer " + get_api_key(),
            "Content-Type": "application/json",
        },
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            suffix = "." + response_format
            fd, output_path = tempfile.mkstemp(prefix="dingo_tts_", suffix=suffix)
            with os.fdopen(fd, "wb") as output_file:
                output_file.write(resp.read())
            return output_path
    except urllib.error.HTTPError as exc:
        body = exc.read().decode("utf-8", "replace")
        raise RuntimeError("OpenAI speech HTTP %s: %s" % (exc.code, body))


def image_to_data_url(image_path):
    content_type = mimetypes.guess_type(image_path)[0] or "image/jpeg"
    with open(image_path, "rb") as image_file:
        encoded = base64.b64encode(image_file.read()).decode("ascii")
    return "data:%s;base64,%s" % (content_type, encoded)
