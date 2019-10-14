import time
import RPi.GPIO as GPIO
from pigpio_encoder import pigpio_encoder
from RpiMotorLib import rpi_dc_lib

class Pendulum():
    
    def __init__(self):
        
        self.setup_pins()
        self.setup_constants()
        self.setup_state_variables()

        self.base_motor = rpi_dc_lib.TranDc(pin = self.motor_enable_pin, freq = self.motor_pwm_freq, verbose = True)

        self.base_rotary = pigpio_encoder.Rotary(clk = self.clk_base_pin, dt = self.dt_base_pin, sw = self.sw_base_pin)
        self.base_rotary.setup_rotary(min = self.motor_rot_min, max = self.motor_rot_max, scale = self.motor_rot_scale, rotary_callback = self.base_callback)
        self.base_rotary.setup_switch(sw_short_callback = self.sw_base_rot_short)

        self.arm_rotary_1 = pigpio_encoder.Rotary(clk = self.clk_pin_1, dt = self.dt_pin_1, sw = self.sw_pin_1)
        self.arm_rotary_1.setup_rotary(min = self.rot_min_1, max = self.rot_max_1, scale = self.rot_scale_1, rotary_callback = self.arm_callback_1)
        self.arm_rotary_1.setup_switch(sw_short_callback = self.sw_arm_rot_short_1)

        self.base_rotary.watch()
        self.arm_rotary_1.watch()
        
        time.sleep(1)
        
        # rotate base to zero position
        self.base_rotate(0.0)
        
        # start main loop for the inverse pendulum
        self.control_loop()
        
    def setup_pins(self):
        # define pins and set their input/output mode
        self.motor_enable_pin = 8
        self.motor_dir_pin = 10
        
        self.clk_base_pin = 23
        self.dt_base_pin = 24
        self.sw_base_pin = 16

        self.clk_pin_1 = 27
        self.dt_pin_1 = 22
        self.sw_pin_1 = 12
        
        GPIO.cleanup()

        GPIO.setmode(GPIO.BCM)

        GPIO.setup(self.motor_enable_pin, GPIO.OUT)
        GPIO.setup(self.motor_dir_pin, GPIO.OUT)
        
        GPIO.setup(self.clk_base_pin, GPIO.IN)
        GPIO.setup(self.dt_base_pin, GPIO.IN)
        GPIO.setup(self.sw_base_pin, GPIO.IN)

        GPIO.setup(self.clk_pin_1, GPIO.IN)
        GPIO.setup(self.dt_pin_1, GPIO.IN)
        GPIO.setup(self.sw_pin_1, GPIO.IN)
        
    def setup_constants(self):
        
        self.motor_step_delay = 0.05
        self.motor_pwm_freq = 200
        self.motor_rot_min = 0
        self.motor_rot_max = 360
        self.motor_rot_scale = 0.1
        
        self.rot_min_1 = 0
        self.rot_max_1 = 360
        self.rot_scale_1 = 0.1
        
    def setup_state_variables(self):
        
        self.last_base_angle = 0.0
        self.last_base_angle_time = 0
        self.base_angle = 0.0
        self.base_velocity = 0.0
        
        self.last_arm_angle_1 = 180.0
        self.last_arm_angle__time_1 = 0
        self.arm_angle_1 = 180.0
        self.arm_angle_velocity_1 = 0.0
        
        self.target_arm_angle_1 = 0.0
        
    def sw_base_rot_short(self):
        
        # do nothing
        pass
        
    def sw_arm_rot_short_1(self):
        
        #do nothing
        pass
        
    def base_callback(self, counter):
        
        self.base_angle = counter
        print("Base angle: ", counter)
        
    def arm_callback_1(self, counter):
        
        self.arm_angle_1 = counter
        print("Arm 1 angle: ", counter)
        
    def control_loop(self):
        
        while True:
            if not self.arm_angle_1 == self.target_arm_angle_1:
                angle = self.arm_angle_1
                print("Rotating: ", angle)
                self.base_rotate(angle)
                
    def base_rotate(self, angle):
        
        while not angle == self.base_angle:
            if self.base_angle < angle:
                set_base_rot_dir(1)
            else:
                set_base_rot_dir(-1)
                
            speed = 100
            self.base_motor.dc_motor_run(speed, self.step_delay)
        
    def set_base_rot_dir(self, dir):
        # dir == -1 is ccw (looking down on pivot) and dir == 1 is cw (again, looking down)
        if dir == 1:
            GPIO.output(self.motor_enable_pin, GPIO.HIGH)
        elif dir == -1:
            GPIO.output(self.motor_enable_pin, GPIO.LOW)
        else:
            print("incorrect way of braking; set speed to 0 instead")
            
            

if __name__ == '__main__':
    
    pendulum = Pendulum()