#!/bin/bash
# @file setup_env.sh
# @brief Single-command environment setup for ProjectHailMary.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install/setup.bash"
export CUAS_ROOT="${SCRIPT_DIR}"
echo "ProjectHailMary environment ready."
echo "  ROS_DISTRO : ${ROS_DISTRO}"
echo "  CUAS_ROOT  : ${CUAS_ROOT}"
