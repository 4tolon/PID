# import time
from machine import Pin, I2C, PWM
from vl53l0x import VL53L0X
from time import sleep_ms
import time

#Servo
def map(x, in_min, in_max, out_min, out_max):
    return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    
class GORILLACELL_SERVO:
    def __init__(self, signal_pin):
        self.pwm = PWM(Pin(signal_pin), freq=50, duty=0)

    def rotate(self, angle):
        self.pwm.duty(map(angle, 0, 180, 23, 124))
        

servo = GORILLACELL_SERVO(signal_pin=25)

# Distanciometro
print("setting up i2c")
sda = Pin(5)
scl = Pin(4)
id1 = 0

i2c = I2C( sda=sda, scl=scl)

print(i2c.scan())


tof = VL53L0X(i2c)

budget = tof.measurement_timing_budget_us
print("Budget was:", budget)
tof.set_measurement_timing_budget(40000)
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 18)

tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 8)

#PID
kp = 18
ki = 2
kd = 8100
distance_setpoint = 200
distance_previous_error = 0
period = 50
PID_i = 0

servo.rotate(0)

print(time.time())

while True:
    distance1 = tof.ping()-50
    distance = tof.ping()-50
    print( 'distance:', distance)
    distance_error = distance_setpoint - distance
    print( 'distance_error:', distance_error)
    PID_p = kp * distance_error
    distance_diference = distance_error - distance_previous_error
    PID_d = kd *(distance_error - distance_previous_error)/period
    print(distance, distance_error, PID_p, distance_diference, PID_d)
    if -3 <distance_error | 3> distance_error:
         PID_i = PID_i +(ki * distance_error)
         print('primer if')
    else:
        PID_i=0
        print('segundo if osea esle')

    PID_total = PID_p + PID_i + PID_d
    print(PID_total, tof.ping()-50)
    if PID_total < 20:
        PID_total = 130
    if PID_total > 160:
        PID_total = 20
    servo.rotate(PID_total+30)
    distance_previous_error = distance_error

#while True:
#    print(tof.ping())

