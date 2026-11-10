#!/bin/bash
# Runs a benchmark campaign (tools/run_batch.py) inside the Docker image.
#
# Usage:
#   docker/run_campaign.sh RESULTS_DIR [run_batch.py arguments]
# Example:
#   docker/run_campaign.sh ~/results --categories 2 --controllers teb mppi rpp --conditions all --seeds 6
#
# The image can be selected with the environment variable IMAGE.
# The simulator renders the sensors with OpenGL: without GPU access, it runs much
# slower than real time and the validity check rejects every run.
set -e

if [ $# -lt 1 ]; then
	sed -n '2,12p' "$0"
	exit 1
fi

IMAGE=${IMAGE:-ghcr.io/fercanara/robotics_benchmark_greenhouse:latest}
RESULTS=$(realpath -m "$1")
shift
mkdir -p "$RESULTS"

GPU_ARGS=()
if command -v nvidia-smi > /dev/null && nvidia-smi -L > /dev/null 2>&1; then
	GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
elif [ -d /dev/dri ]; then
	GPU_ARGS=(--device /dev/dri)
else
	echo "WARNING: no GPU found: the runs will probably not keep real time." >&2
fi

# Interactive (Ctrl+C stops the campaign) only when run from a terminal:
TTY_ARGS=()
if [ -t 0 ]; then
	TTY_ARGS=(-it)
fi

TOOLS=/ws/src/robotics_benchmark_greenhouse/benchmark_bringup/benchmark_bringup/tools
exec docker run --rm "${TTY_ARGS[@]}" "${GPU_ARGS[@]}" \
	--user "$(id -u):$(id -g)" -e HOME=/tmp \
	-e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
	-v "$RESULTS":/results \
	"$IMAGE" python3 "$TOOLS/run_batch.py" --out /results "$@"
