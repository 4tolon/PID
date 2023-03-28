# import time
from machine import Pin, SoftI2C, PWM
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

i2c = SoftI2C( sda=sda, scl=scl)

print(i2c.scan())


tof = VL53L0X(i2c)

budget = tof.measurement_timing_budget_us
print("Budget was:", budget)
tof.set_measurement_timing_budget(40000)
tof.set_Vcsel_pulse_period(tof.vcsel_period_type[0], 18)

tof.set_Vcsel_pulse_period(tof.vcsel_period_type[1], 8)

#PID
kp = -0.6#0.7


ki = 0.02
kd = -150 #-1200
distance_setpoint =150
distance_previous_error = 0
period = 20

PID_p = 0
PID_i = 0
PID_d = 0
PID_total = 0

timenow = 0
previous_error=0

#servo.rotate(35)

#print(time.time())
#time_n = time.ticks_ms()
#print(time_n)

while True:
    distance1 = tof.ping()-50
    distance = tof.read()-50
    print( 'distance:', distance)
    distance_error = distance_setpoint - distance
    print('distance_error:',distance_error)
#    print( 'distance_error:', distance_error)
    # PROPORCIONAL
    PID_p = kp * distance_error
    print('PID_p',PID_p)
    #print(distance, distance_error, PID_p, distance_diference, PID_d)
    # INTEGRAL
    if -10 <distance_error | 400> distance_error:
         PID_i = PID_i -(ki * distance_error)
    print('PID_i', PID_i)
         #print('primer if')
        #print('segundo if osea esle')
    #DERIVATIVE
    #distance_diference = distance_error - distance_previous_error
    #PID_d = kd *(distance_error - distance_previous_error)/period
    time_previous=timenow
   
    timenow=time.ticks_ms()
    elapsedTime=timenow-time_previous
    
    #if elapsedTime == 0:
     #   elapsedTime= 1
    print('elapsedTime', elapsedTime)
    PID_d=kd*((distance_error-previous_error)/elapsedTime)
    print('PID_d', PID_d)
    previous_error=distance_error

    PID_total = PID_p + PID_i + PID_d
    print(PID_total, PID_p, PID_i, PID_d)
    servo_signal = PID_total
    
    #print(PID_total, "-----", PID_p, PID_i, PID_d)
    #print(PID_total, tof.ping()-260)
    if servo_signal < 70:
        servo_signal = 70
    if servo_signal > 150:
        servo_signal = 150
        
    print('servo_signal: ',servo_signal)
    
    servo.rotate(servo_signal)
    
    distance_previous_error = distance_error
    time.sleep_ms(100)
    
    