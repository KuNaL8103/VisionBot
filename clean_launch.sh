#!/bin/bash
echo "Cleaning up any leftover processes..."
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ros_gz" 2>/dev/null
pkill -9 -f "parameter_bridge" 2>/dev/null
pkill -9 -f "robot_state_publisher" 2>/dev/null
pkill -9 -f "perception_node" 2>/dev/null
pkill -9 -f "controller_node" 2>/dev/null
sleep 2
echo "Done. Process list:"
ps aux | grep -E "gz sim|ros_gz|perception_node|controller_node" | grep -v grep
