#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

sudo docker build \
    -t dingo-vision:latest \
    -f "${SCRIPT_DIR}/Dockerfile" \
    "${PROJECT_ROOT}"
