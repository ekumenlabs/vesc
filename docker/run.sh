#!/bin/bash

# Copyright 2026 Ekumen, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

set -o errexit

ROS_DISTRO=jazzy

cd "$(dirname "$(readlink -f "$0")")"

[[ ! -z "${WITHIN_DEV}" ]] && echo "Already in the development environment!" && exit 1

# function to print help options
function print_help {
    echo "Usage: $(basename $0) [-b|--build] [-h|--help] [-- COMMAND [ARGS...]]"
    echo ""
    echo "Options:"
    echo "  --build                      Build the image before starting the container."
    echo "  --help                       Display this help message."
    echo "  --                           Pass everything after this as command and arguments to docker run."
    echo ""
    echo "Examples:"
    echo "  $(basename $0)                          # Run default container"
    echo "  $(basename $0) -- bash                  # Run bash in container"
    echo "  $(basename $0) -- ros2 topic list       # Run ROS 2 command"
    echo "  $(basename $0) --build -- bash          # Build image then run bash"
}

set +o errexit
VALID_ARGS=$(OPTERR=1 getopt -o bh --long build,help -- "$@")
RET_CODE=$?
set -o errexit

if [[ $RET_CODE -eq 1 ]]; then
    print_help
    exit 1;
fi
if [[ $RET_CODE -ne 0 ]]; then
    >&2 echo "Unexpected getopt error"
    exit 1;
fi

SERVICE_NAME="vesc_devel"
DOCKER_COMMAND=()

eval set -- "$VALID_ARGS"
while [[ "$1" != "" ]]; do
    case "$1" in
    -b | --build)
        BUILD=true
        shift
        ;;
    -h | --help)
        print_help
        exit 0
        ;;
    --) # start of positional arguments
        shift
        # Capture remaining arguments as docker command
        DOCKER_COMMAND=("$@")
        break
        ;;
    *)
        >&2 echo "Unrecognized positional argument: $1"
        print_help
        exit 1
        ;;
    esac
done

# if the ROS_DOMAIN_ID variable is not set, abort
if [[ -z "${ROS_DOMAIN_ID}" ]]; then
  echo
  echo "The ROS_DOMAIN_ID environment variable must be set."
  echo
  exit 1
fi

CONTAINER_NAME="${USER}_${SERVICE_NAME}"

# Build compose file arguments
COMPOSE_FILES="-f docker-compose.yml"

if [[ "$BUILD" = true ]]; then
    USERID=$(id -u) GROUPID=$(id -g) ROS_DISTRO=${ROS_DISTRO} \
        docker compose ${COMPOSE_FILES} build ${SERVICE_NAME} \
    && echo "Built ${SERVICE_NAME} image." || \
        { echo "Failed to build ${SERVICE_NAME} image."; exit 1; }
fi

USERID=$(id -u) GROUPID=$(id -g) ROS_DISTRO=${ROS_DISTRO} \
    docker compose ${COMPOSE_FILES} run \
    --name "${CONTAINER_NAME}" \
    --remove-orphans \
    --rm "${SERVICE_NAME}" \
    "${DOCKER_COMMAND[@]}"
