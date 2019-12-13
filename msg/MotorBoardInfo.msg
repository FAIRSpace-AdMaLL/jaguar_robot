#this is used for Motor Driver board info
Header header 	  #std_msg header
uint32 status	  # as described in motor driver board manual
float32 temp1	  # temperature 1, internal chip temperature
float32 temp2	  # driver channel 1 temperature
float32 temp3	  # driver channel 2 temperature
float32 volMain  # main power voltage, default is battery voltage
float32 vol12V	  # 12V power 
float32 vol5V	  # 5V power
uint32 dinput	  # digital input, not used now
uint32 doutput	  # digital output, not used now
int32 ack	  # 0- right command received(receive "+") -1 wrong("-")
