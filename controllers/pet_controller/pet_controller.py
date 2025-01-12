from controller import Robot, Camera, Motor, PositionSensor
import numpy as np
import math

class PetRobot(Robot):
    def __init__(self):
        super(PetRobot, self).__init__()
        
        # Initialize devices
        self.timestep = int(self.getBasicTimeStep())
        
        # Initialize camera
        self.camera = self.getDevice('camera')
        self.camera.enable(self.timestep)
        
        # Initialize motors
        self.tail_motor = self.getDevice('tail_motor')
        self.legs = {
            'fr': self.getDevice('front_right_leg'),
            'fl': self.getDevice('front_left_leg'),
            'br': self.getDevice('back_right_leg'),
            'bl': self.getDevice('back_left_leg')
        }
        
        # Initialize tail position sensor
        self.tail_sensor = self.getDevice('tail_sensor')
        self.tail_sensor.enable(self.timestep)
        
        # Set motor parameters
        self.tail_motor.setPosition(float('inf'))
        self.tail_motor.setVelocity(0.0)
        
        for leg in self.legs.values():
            leg.setPosition(float('inf'))
            leg.setVelocity(0.0)
        
        # States
        self.is_jumping = False
        self.jump_start_time = 0
        self.tail_wiggling = False
        
        # Image processing parameters
        self.prev_image = None
        self.movement_threshold = 1000  # Adjust based on testing
    
    def detect_motion(self):
        """Detect motion using simple frame differencing"""
        if self.prev_image is None:
            self.prev_image = np.array(self.camera.getImageArray())
            return False
        
        current_image = np.array(self.camera.getImageArray())
        diff = np.abs(current_image - self.prev_image).sum()
        
        self.prev_image = current_image
        return diff > self.movement_threshold
    
    def wiggle_tail(self):
        """Make the tail wiggle in a sinusoidal pattern"""
        current_time = self.getTime()
        angle = math.sin(current_time * 5.0) * 0.5
        self.tail_motor.setPosition(angle)
    
    def jump(self):
        """Initiate a jump sequence"""
        if not self.is_jumping:
            self.is_jumping = True
            self.jump_start_time = self.getTime()
            # Set leg motors for jump
            for leg in self.legs.values():
                leg.setPosition(0.5)  # Compress legs
                leg.setVelocity(10.0)
    
    def update_jump(self):
        """Update the jumping state"""
        if self.is_jumping:
            current_time = self.getTime()
            if current_time - self.jump_start_time > 0.2:  # Jump duration
                # Reset legs
                for leg in self.legs.values():
                    leg.setPosition(0.0)
                    leg.setVelocity(5.0)
                self.is_jumping = False
    
    def run(self):
        """Main control loop"""
        while self.step(self.timestep) != -1:
            # Check for motion
            if self.detect_motion():
                self.tail_wiggling = True
                print("Motion detected! Wiggling tail...")
            
            # Update tail movement
            if self.tail_wiggling:
                self.wiggle_tail()
            
            # Periodically trigger jump for demo purposes
            # You can modify this based on your needs
            current_time = self.getTime()
            if int(current_time) % 10 == 0:  # Jump every 10 seconds
                self.jump()
            
            # Update jumping state
            self.update_jump()

# Create and run the robot
robot = PetRobot()
robot.run()