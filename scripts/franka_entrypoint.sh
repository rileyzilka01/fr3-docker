#!/bin/bash

# Clone Franka dependencies into the workspace
vcs import /franka_ws/src < /franka_ws/src/franka.repos --recursive --skip-existing

exec "$@"