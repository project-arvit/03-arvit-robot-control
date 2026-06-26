#!/usr/bin/env bash
# Apple `container` path for arvit-robot-control (single-container; no compose).
# Builds the image from the SAME Dockerfile as the Docker path, then runs it.
#
# By default it runs the pure-logic test suite (extrinsics + timesync). Pass
# extra args to override, e.g.:
#   scripts/container-up.sh python -c "from arvit_robot_control.slam.extrinsics import HeadMount; print(HeadMount().to_static_transform_publisher_args())"
#
# The ROS 2 launch needs the real robot + L1; it is a deploy-on-Orin step, not
# run here. No published ports, no host network. Secrets (if ever needed) go
# via --env-file .env only (.env is gitignored).
set -euo pipefail

IMAGE="arvit-robot-control:latest"
NAME="arvit-robot-control"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

cd "$HERE"

# Secrets ONLY via --env-file (.env is gitignored). Use .env if present.
ENV_ARGS=()
if [[ -f .env ]]; then
  ENV_ARGS=(--env-file .env)
fi

echo ">> container build -t ${IMAGE}"
container build -t "${IMAGE}" -f Dockerfile .

# Remove any prior instance with the same name so re-runs are clean.
container rm "${NAME}" >/dev/null 2>&1 || true

echo ">> container run --name ${NAME}"
# --rm so the one-shot run cleans itself up; pass-through args override CMD.
container run --rm --name "${NAME}" ${ENV_ARGS[@]+"${ENV_ARGS[@]}"} "${IMAGE}" "$@"
