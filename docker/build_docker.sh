#!/bin/bash

IMAGE_NAME="ros2-humble"

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
PROJECT_DIR="$(dirname "${SCRIPT_DIR}")"
TAG="ros2-event-camera"
DOCKERFILE="${SCRIPT_DIR}/Dockerfile"

if [ "${#}" -gt "0" ]; then
    if [[ "${1}" != "-"* ]]; then
        TAG="${TAG}:${1}"
        BUILD_ARGS=${*:2}
    else
        BUILD_ARGS=${*:1}
    fi
fi

DOCKER_BUILD_CMD=(
    docker build
    -t $IMAGE_NAME
    -f "${DOCKERFILE}"
    "${PROJECT_DIR}"
    --tag ${TAG}
    #"${BUILD_ARGS}"
)

echo -e "\033[0;32m${DOCKER_BUILD_CMD[*]}\033[0m" | xargs

# shellcheck disable=SC2068
exec ${DOCKER_BUILD_CMD[*]}