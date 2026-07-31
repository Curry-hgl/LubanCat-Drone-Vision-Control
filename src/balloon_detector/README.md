ros2 run balloon_detector rknn_detector_node --ros-args \
  -p device_path:=/dev/video0 \
  -p model_path:=/home/cat/ultralytics_yolo11/yolo11n.rknn \
  -p conf_threshold:=0.5 \
  -p target_class:=-1 \
  -p video_output:=true