from hexapod.Ax18 import Ax18


Ax18.DEVICENAME = '/dev/ttyUSB0'
Ax18.BAUDRATE = 500000
Ax18.connect()


def setup_motors(self):
    """Настройка параметров моторов"""
    for motor in [self.motor_coxa, self.motor_femur, self.motor_tibia]:
        motor.set_torque_enable(200)
        motor.set_moving_speed(100)