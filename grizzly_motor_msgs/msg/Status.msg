# CAN ID of the device.
uint8 device_number

# Name of joint controlled, or other identifier.
string device_name

# Temperature readings from the driver and motor (degC).
float32 temperature_driver
float32 temperature_motor

# Input terminal voltage to the driver (volts).
float32 voltage_input

# Voltage outputed to the motor (volts).
float32 voltage_output

# Current being consumed by the motor (amps).
float32 current
