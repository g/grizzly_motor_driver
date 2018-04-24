# CAN ID of the device.
uint8 device_number

# Name of joint controlled, or other identifier.
string device_name

# Commanded heading of the controller [-100,100].
float32 heading

# Travel as measured by the encoder (rad).
float64 travel

# Angular velocity as measured by the encoder (rad/s).
float64 velocity
