cmake_minimum_required(VERSION 3.0.2)
project(auto_control)

# 依存する catkin パッケージを指定
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)

# catkin 特有の設定
catkin_package(
  # CATKIN_DEPENDS rospy sensor_msgs std_msgs
)

# ヘッダーファイルのインクルードディレクトリ（C++の場合）
include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Python スクリプトのインストール
catkin_install_python(PROGRAMS
  scripts/auto_controller.py
  scripts/sensor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
