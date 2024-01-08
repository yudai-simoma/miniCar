rosbag record  --duration=6m  -b 2048  -o /home/ubuntu/sdcard/bags/with_imu_depth_registered_points \
	/joy \
	/camera/depth_registered/points \
	/camera/aligned_depth_to_color/image_raw \
	/camera/gyro/sample /camera/accel/sample \
	 --lz4
