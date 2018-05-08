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

# Start-up errors that would stop the unit from being configured.
uint16 ERROR_STARTUP_MIN_GREATER_THAN_MAX = 1
uint16 ERROR_STARTUP_MAX_GREATER_THAN_MIN = 2
uint16 ERROR_STARTUP_THROTTLE_RANGE_TOO_SMALL = 4
uint16 ERROR_STARTUP_THROTTLE_RANGE_TOO_LARGE = 8
uint16 ERROR_STARTUP_CURRENT_SENSOR_CALIBRATION = 16
uint16 ERROR_STARTUP_HW_SHUT_DOWN = 32
uint16 ERROR_STARTUP_WATCH_DOG_RESET = 64
uint16 ERROR_STARTUP_ERRORS = 128
uint16 error_startup
