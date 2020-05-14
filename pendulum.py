# you are going to have to download at least one of these libraries (probably encoder)
from RPi import GPIO
from time import sleep
import numpy as np
from Encoder.Encoder import Encoder


class Pendulum:
    
    def __init__(self):
        # rotary encoder's pulses per rotation (resolution)
        self.pulse_per_rotation = 1024.0
        self.setpoint = self.pulse_per_rotation / 2.0
        
        self.motor_enable = 22 # pwm output for motor driver
        self.motor_ina = 20 # output 1 for clockwise, 0 for counter-clockwise
        self.motor_inb = 21 # output 0 for clockwise, 1 for counter-clockwise
        
        self.enc = Encoder(17, 27)
        
        # set mode to BCM so GPIO pin numbers are used instead of absolute
        GPIO.setmode(GPIO.BCM)
        
        GPIO.setup(self.motor_enable, GPIO.OUT) # output for motor pwm
        GPIO.setup(self.motor_ina, GPIO.OUT) # output for motor ina
        GPIO.setup(self.motor_inb, GPIO.OUT) # output for motor inb
        
        self.motor_pwm = GPIO.PWM(self.motor_enable, 60) # 60 hertz frequency (i think)
        self.motor_pwm.start(0) # start with 0% duty cycle (0-100 is the same as 0-255 in arduino)
        
        self.clockwise(0) # initially clockwise

        self.counter = 0 # initially 0 counter, pendulum always starts down
        
    def update_encoder(self):
        self.counter = self.enc.read()
        
    def run_motor(self, speed):
        if speed < 0:
            self.counter_clockwise(speed) # move motor counter clockwise
        elif speed > 0:
            self.clockwise(speed) # move motor clockwise
        else:
            self.motor_pwm.ChangeDutyCycle(0) # brake motor
            
    def break_motor(self):
        GPIO.output(self.motor_ina, 1)
        GPIO.output(self.motor_inb, 1)
        GPIO.output(self.motor_ina, 0)
        GPIO.output(self.motor_inb, 0)
            
    def clear_error(self):
        self.motor_pwm.ChangeDutyCycle(0)
        GPIO.output(self.motor_ina, 0)
        GPIO.output(self.motor_inb, 0)
        GPIO.output(self.motor_ina, 1)
        GPIO.output(self.motor_inb, 1)
        GPIO.output(self.motor_ina, 0)
        GPIO.output(self.motor_inb, 0)
        self.clockwise(0)
            
    def clockwise(self, speed):
        GPIO.output(self.motor_ina, 1) # output 1 to ina for clockwise
        GPIO.output(self.motor_inb, 0) # output 0 to inb for counter clocwise
        speed = np.abs(speed) # get absolute value for outputting as duty cycle
        # bind value to below 100% duty cycle (can't have >100% duty cycle)
        if speed > 100: 
            speed = 100
        self.motor_pwm.ChangeDutyCycle(speed)
        
    def counter_clockwise(self, speed):
        GPIO.output(self.motor_ina, 0) # output 0 to ina for counter clockwise
        GPIO.output(self.motor_inb, 1) # output 1 to inb for clockwise
        speed = np.abs(speed) # get absolute value for outputting as duty cycle
        # bind value to below 100% duty cycle (can't have >100% duty cycle)
        if speed > 100:
            speed = 100
        self.motor_pwm.ChangeDutyCycle(speed)
        
    def swing_up(self):
        # do swing up routine to begin
        # couldn't figure out how to do this with the encoder not working
        # so, WIP
        
        self.run_motor(50)
        sleep(0.177)
        self.run_motor(-50)
        sleep(0.1785)
        self.run_motor(20)
        sleep(0.1)
        self.break_motor()
        
        # todo: implement swing up
        
        
    def run(self):
#         self.swing_up()
        
        try:
            # initialize some things before starting loop
            error = 0
            last_error = 0
            sum_error = 0
            
            # constants for PID control system
            kP = 0.025 # proportional
            kD = 0.0 # derivative
            kI = 0.0 # i don't anticipate this being needed, but this is for integral
            
            while True:
                self.update_encoder() # update self.counter variable
                
                self.swing_up()
                sleep(7.0)
                
                #if self.counter < 90:
#                     self.swing_up()
                    #print('not implemented')
                
                # get difference between current position and where we want it to be (WIP)
                error = self.counter - self.setpoint
                
                # shoud be between 0 and 100, because it directly corresponds to duty cycle of motor
                adjustment = (kP * error) + (kD * (last_error - error)) + (kI * sum_error)
                
                last_error = error # update last_error
                sum_error = sum_error + error # update sum_error
#                 print(self.counter)
#                 self.run_motor(adjustment) # run motor at pwm required to adjust for current error
                print(self.counter)
#                 self.run_motor(10)
#                 self.clear_error()
        except KeyboardInterrupt:
            print('interrupted')
        finally:
            GPIO.cleanup()

pendulum = Pendulum()

pendulum.run()
