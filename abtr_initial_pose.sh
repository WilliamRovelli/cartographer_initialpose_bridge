#!/bin/bash
set -e

#######################################
# 1. 配置区域（只需操作这个区域）
#######################################

# Cartographer 工程所在 workspace
CARTO_WS=/home/wr/cartographer

# Cartographer 定位用的 lua 文件名
CONFIG_BASENAME=backpack_2d_localization.lua

# run/ 里 rviz & pbstream 的“基名”
#   会自动拼成  abtr_rviz.rviz  和  map1.pbstream
RVIZ_NAME=abtr_rviz
MAP_NAME=map1

# 播 pbstream 的时间（秒），给 RViz 足够时间接收地图
SLEEP_BEFORE_KILL=5

#######################################
# 2. 根据脚本所在位置推导 workspace / run 路径
#######################################

# abtr_init 工作空间根目录（脚本所在目录）
WS_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ABTR_WS="$WS_DIR"

RUN_DIR="$WS_DIR/src/abtr_initial_pose/run"
PBSTREAM_FILE="$RUN_DIR/${MAP_NAME}.pbstream"
RVIZ_FILE="$RUN_DIR/${RVIZ_NAME}.rviz"

# Cartographer 配置目录（按cartographer源码结构）
CONFIG_DIR="$CARTO_WS/src/cartographer_ros/cartographer_ros/configuration_files"

echo "[abtr_initial_pose] WS_DIR       = $WS_DIR"
echo "[abtr_initial_pose] RUN_DIR      = $RUN_DIR"
echo "[abtr_initial_pose] PBSTREAM_FILE= $PBSTREAM_FILE"
echo "[abtr_initial_pose] RVIZ_FILE    = $RVIZ_FILE"
echo "[abtr_initial_pose] CONFIG_DIR   = $CONFIG_DIR"

# 基本检查：pbstream & rviz 文件是否存在
if [ ! -f "$PBSTREAM_FILE" ]; then
  echo "[abtr_initial_pose] ERROR: pbstream file not found: $PBSTREAM_FILE"
  exit 1
fi

if [ ! -f "$RVIZ_FILE" ]; then
  echo "[abtr_initial_pose] ERROR: RViz config not found: $RVIZ_FILE"
  exit 1
fi

#######################################
# 用 Cartographer 环境在子 shell 里起 pbstream_map_publisher
#######################################

(
  source "$CARTO_WS/devel_isolated/setup.bash"
  rosrun cartographer_ros cartographer_pbstream_map_publisher \
    -pbstream_filename="$PBSTREAM_FILE" \
    -resolution=0.05
) &
PB_PID=$!

echo "[abtr_initial_pose] cartographer_pbstream_map_publisher started, pid=$PB_PID"

#######################################
#  source 自己 workspace，打开 RViz
#######################################

source "$ABTR_WS/devel/setup.bash"

rviz -d "$RVIZ_FILE" &
RVIZ_PID=$!
echo "[abtr_initial_pose] rviz started, pid=$RVIZ_PID"

# 给 RViz 一点时间接收地图
sleep "$SLEEP_BEFORE_KILL"

# 关掉 pbstream 发布节点（当前 RViz 窗口里的地图会保留）
echo "[abtr_initial_pose] killing cartographer_pbstream_map_publisher (pid=$PB_PID)..."
kill "$PB_PID" 2>/dev/null || true

#######################################
# 5. 启动 initialpose bridge（前台运行）
#######################################

echo "[abtr_initial_pose] starting cartographer_initialpose_bridge via roslaunch..."
roslaunch abtr_initial_pose abtr_initial_pose.launch \
  config_dir:="$CONFIG_DIR" \
  config_basename:="$CONFIG_BASENAME"

