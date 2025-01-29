import board
import busio
import adafruit_bno055

i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

print("Temperature:", sensor.temperature)
print("Euler angle:", sensor.euler)
print("Accelerometer (m/s^2):", sensor.acceleration)
print("Gyroscope (rad/s):", sensor.gyro)
print("Magnetometer (microteslas):", sensor.magnetic)
print("Linear acceleration (m/s^2):", sensor.linear_acceleration)
print("Gravity (m/s^2):", sensor.gravity)
print("Quaternion:", sensor.quaternion)
print("Calibration status:", sensor.calibration_status)
print("Current mode:", sensor.mode)
print("Calibration status (sys, gyro, accel, mag):", sensor.calibration_status)

