from RPi import GPIO
from time import sleep
import numpy as np


class Pendulum:
    
    def __init__(self):
        self.motor_enable = 22 # pwm output for motor driver
        self.motor_ina = 23 # output 1 for clockwise, 0 for counter-clockwise
        self.motor_inb = 24 # output 0 for clockwise, 1 for counter-clockwise
        
        self.phase_a = 17 # input of phase A for encoder
        self.phase_b  = 27 # input of phase B for encoder
        
        # set mode to BCM so GPIO pin numbers are used instead of absolute
        GPIO.setmode(GPIO.BCM)
        
        GPIO.setup(self.motor_enable, GPIO.OUT) # output for motor pwm
        GPIO.setup(self.motor_ina, GPIO.OUT) # output for motor ina
        GPIO.setup(self.motor_inb, GPIO.OUT) # output for motor inb
        
        GPIO.setup(self.phase_a, GPIO.IN, pull_up_down=GPIO.PUD_UP) # input for encoder phase A
        GPIO.setup(self.phase_b, GPIO.IN, pull_up_down=GPIO.PUD_UP) # input for encoder phase B
        
        self.motor_pwm = GPIO.PWM(self.motor_enable, 60) # 60 hertz frequency (i think)
        self.motor_pwm.start(0) # start with 0% duty cycle (0-100 is the same as 0-255 in arduino)
        
        self.clockwise(0) # initially clockwise

        self.counter = 0 # initially 0 counter, pendulum always starts down
        self.a_state = GPIO.input(self.phase_a) # initially input clkState
        self.a_last_state = GPIO.input(self.phase_a) # initially input clkLastState
        
    def read_encoder(self):
        self.a_state = GPIO.input(self.phase_a) # get current phase A state (0 or 1)
        if self.a_state != self.a_last_state: # if phase A state has changed since last read
            self.b_state = GPIO.input(self.phase_b) # get phase B state (also 0 or 1)
            if self.b_state != self.a_state: # if they are different, encoder is moving clockwise
                self.counter += 1 # increment counter for clockwise
            else: # if they are the same, encoder is moving counter clockwise
                self.counter -= 1 # decrement counter for counter clockwise
            print(self.counter) # print for debugging
        self.a_last_state = self.a_state # update last phase A state for next time around
        # sleep(0.01) # sleep to let encoder input update
        
    def run_motor(self, speed):
        if speed < 0:
            self.counter_clockwise(speed) # move motor counter clockwise
        elif speed > 0:
            self.clockwise(speed) # move motor clockwise
        else:
            self.motor_pwm.ChangeDutyCycle(0) # brake motor
            
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
                
        while self.counter > 5 or self.counter < -5
            sleep(0.01) # wait for pendulum to settle
        # todo: implement swing up
        
        
    def run(self):
        self.swing_up()
        
        try:
            # initialize some things before starting loop
            error = 0
            last_error = 0
            sum_error = 0
            
            # constants for PID control system
            kP = 1.0 # proportional
            kD = 1.0 # derivative
            kI = 0.0 # i don't anticipate this being needed, but this is for integral
            
            setpoint = 180 # doesn't work because encoder currently drifts
            
            while True:
                self.read_encoder() # update self.counter variable
                
                if counter < 90:
#                     self.swing_up()
                
                # get difference between current position and where we want it to be (WIP)
                error = self.counter - setpoint
                
                # shoud be between 0 and 100, because it directly corresponds to duty cycle of motor
                adjustment = (kP * error) + (kD * (last_error - error)) + (kI * sum_error)
                
                last_error = error # update last_error
                sum_error = sum_error + error # update sum_error
                
                self.run_motor(adjustment) # run motor at pwm required to adjust for current error
        finally:
            GPIO.cleanup()

pendulum = Pendulum()

pendulum.run()
