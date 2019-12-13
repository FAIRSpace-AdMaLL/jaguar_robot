#motor sensor data message from Dr Robot 
Header header 	   #timestamp in the header is the time the driver
int64 motorPower	   # motor driver power from motor driver board
int64 encoderPos	   #encoder position reading
int64 encoderVel	   #encoder velocity reading
int64 encoderDiff	   #encoder position differnece reading referee last reading
float32 motorTemp   #motor temperature reading
float32 motorCurrent	  #motor current feedback reading