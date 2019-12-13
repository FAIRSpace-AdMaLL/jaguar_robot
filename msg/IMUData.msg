#IMU sensor data
Header header		# std_msg header
uint32 seq		# package sequency number, 0 ~ 255
float32 yaw		# yaw estimate, radian
float32 pitch		# pitch estimate, radian
float32 roll		# roll estiamte, radian
int32 gyro_x		# raw gyro x axis data
int32 gyro_y		# raw gyro y axis data
int32 gyro_z		# raw gyro z axis data
int32 accel_x		# raw accel x axis data
int32 accel_y		# raw accel y axis data
int32 accel_z		# raw accel z axis data
int32 comp_x		# raw magnetic x axis data
int32 comp_y		# raw magnetic y axis data
int32 comp_z		# raw magnetic z axis data 