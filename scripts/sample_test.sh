#!/usr/bin/env bash
# Simple test script to publish a sequence of robot commands
# Usage: ./test_commands.sh  (make executable with `chmod +x scripts/test_commands.sh`)

set -euo pipefail

TOPIC="/robot_command"
MSG_TYPE="std_msgs/msg/String"

publish() {
	local data="$1"
	ros2 topic pub --once "$TOPIC" "$MSG_TYPE" "data: '$data'"
}

# sequence of commands
publish "PLACE 1,2,EAST"
sleep 0.1
publish "MOVE"
sleep 0.1
publish "MOVE"
sleep 0.1
publish "LEFT"
sleep 0.1
publish "MOVE"
sleep 0.1
publish "REPORT"

echo "Commands published to $TOPIC"

