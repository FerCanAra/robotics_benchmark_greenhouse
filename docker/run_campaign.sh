#!/bin/bash
# Runs a benchmark campaign (tools/run_batch.py) inside the Docker image.
#
# Usage:
#   docker/run_campaign.sh RESULTS_DIR [run_batch.py arguments]
# Example:
#   docker/run_campaign.sh ~/results --categories 2 --controllers teb mppi rpp --conditions all --seeds 6
#
# The image can be selected with the environment variable IMAGE.
# GUI=1 shows the MVSim window, the real-time plots and RViz (run "xhost +local:" first);
# add WITHOUT_RVIZ=1 to leave RViz out.
# FOXGLOVE=1 runs foxglove_bridge, reachable at ws://localhost:${FOXGLOVE_PORT:-8765}.
# The simulator renders the sensors with OpenGL: without GPU access, it runs much
# slower than real time and the validity check rejects every run.
set -e

if [ $# -lt 1 ]; then
	sed -n '2,14p' "$0"
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
	# The container user needs the groups that own the devices (video, render):
	GPU_ARGS=(--device /dev/dri)
	for gid in $(stat -c %g /dev/dri/card* /dev/dri/renderD* 2> /dev/null | sort -u); do
		GPU_ARGS+=(--group-add "$gid")
	done
else
	echo "WARNING: no GPU found: the runs will probably not keep real time." >&2
fi

# Optional GUI: windows on the X server of the desktop.
GUI_ARGS=()
LAUNCH_ARGS=()
if [ -n "$GUI" ] && [ "$GUI" != "0" ]; then
	if [ -z "$DISPLAY" ]; then
		echo "ERROR: GUI requested but DISPLAY is not set." >&2
		exit 1
	fi
	GUI_ARGS=(-e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:ro)
	LAUNCH_ARGS=(--arg headless:=False --arg plot:=True)
	if [ -z "$WITHOUT_RVIZ" ] || [ "$WITHOUT_RVIZ" == "0" ]; then
		LAUNCH_ARGS+=(--arg use_rviz:=True)
	fi
fi

# Optional Foxglove bridge, with its port published on the host:
PORT_ARGS=()
if [ -n "$FOXGLOVE" ] && [ "$FOXGLOVE" != "0" ]; then
	FOXGLOVE_PORT=${FOXGLOVE_PORT:-8765}
	PORT_ARGS=(-p "$FOXGLOVE_PORT:$FOXGLOVE_PORT")
	LAUNCH_ARGS+=(--arg use_foxglove:=True --arg foxglove_port:="$FOXGLOVE_PORT")
fi

# Interactive (Ctrl+C stops the campaign) only when run from a terminal:
TTY_ARGS=()
if [ -t 0 ]; then
	TTY_ARGS=(-it)
fi

TOOLS=/ws/src/robotics_benchmark_greenhouse/benchmark_bringup/benchmark_bringup/tools
exec docker run --rm "${TTY_ARGS[@]}" "${GPU_ARGS[@]}" "${GUI_ARGS[@]}" "${PORT_ARGS[@]}" \
	--user "$(id -u):$(id -g)" -e HOME=/tmp \
	-e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}" \
	-v "$RESULTS":/results \
	"$IMAGE" python3 "$TOOLS/run_batch.py" --out /results "${LAUNCH_ARGS[@]}" "$@"
