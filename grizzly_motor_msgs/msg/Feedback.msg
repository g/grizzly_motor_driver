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

# The targeted angular velocity (rad/s).
float64 velocity_target

# Runtime errors that would cause the driver to halt.
uint16 ERROR_RUNTIME_BRAKE_OVER_CURRENT = 1
uint16 ERROR_RUNTIME_HW_SHUTDOWN = 2
uint16 ERROR_RUNTIME_POT1_OPEN = 4
uint16 ERROR_RUNTIME_POT2_OPEN = 8
uint16 ERROR_RUNTIME_BOTH_DIR_SWITCHES = 16
uint16 ERROR_RUNTIME_OVER_TEMP = 32
uint16 ERROR_RUNTIME_CURRENT_SENSOR_SATURATED = 64
uint16 ERROR_RUNTIME_BATTERY_OVER_VOLTAGE_ON_START = 128
uint16 ERROR_RUNTIME_BATTERY_UNDER_VOLTAGE_ON_START = 256
uint16 ERROR_RUNTIME_BASE_DISCHARGED = 512
uint16 error_runtime
