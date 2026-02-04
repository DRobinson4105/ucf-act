#!/usr/bin/env sh

if [ -n "${ACT_ROS_WS:-}" ] && [ -f "${ACT_ROS_WS}/.env" ]; then
    set -a
    . "${ACT_ROS_WS}/.env"
    set +a
fi
