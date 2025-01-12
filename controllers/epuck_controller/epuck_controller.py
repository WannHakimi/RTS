from controller import Robot, Motor, LED, DistanceSensor
import time

class EPuckController(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())
        
        # Initialize LEDs
        self.leds = []
        for i in range(8):
            led = self.getDevice(f'led{i}')
            if led:
                self.leds.append(led)
        
        # Initialize proximity sensors
        self.ps = []
        for i in range(8):
            sensor = self.getDevice(f'ps{i}')
            if sensor:
                sensor.enable(self.timestep)
                self.ps.append(sensor)
        
        # LED timer dictionary
        self.led_timers = {i: 0 for i in range(len(self.leds))}
        self.LED_DURATION = 3000  # 3 seconds in milliseconds

        # Direction mapping for better debug messages
        self.direction_names = {
            0: "FRONT",
            1: "FRONT-RIGHT",
            2: "RIGHT",
            3: "BACK-RIGHT",
            4: "BACK",
            5: "BACK-LEFT",
            6: "LEFT",
            7: "FRONT-LEFT"
        }

    def check_collisions(self):
        """Check for very close objects using distance sensors"""
        threshold = 1000  # Very high value indicates very close object
        collisions = []
        
        for i, sensor in enumerate(self.ps):
            value = sensor.getValue()
            # Print sensor values for debugging
            #print(f"Sensor {i} ({self.direction_names[i]}): {value}")
            
            if value > threshold:
                print(f"Very close object detected at {self.direction_names[i]}")
                collisions.append(i)
                self.blink_led(i)
        
        return collisions

    def blink_led(self, led_index):
        """Blink a specific LED"""
        if 0 <= led_index < len(self.leds):
            current_time = self.getTime() * 1000
            self.leds[led_index].set(1)
            self.led_timers[led_index] = current_time
            print(f"LED {led_index} turned ON ({self.direction_names[led_index]} position)")

    def update_leds(self):
        """Turn off LEDs after duration"""
        current_time = self.getTime() * 1000
        for led_index, start_time in self.led_timers.items():
            if start_time > 0 and current_time - start_time > self.LED_DURATION:
                self.leds[led_index].set(0)
                self.led_timers[led_index] = 0
                print(f"LED {led_index} turned OFF ({self.direction_names[led_index]} position)")

    def run(self):
        # Initial delay to let sensors stabilize
        for _ in range(10):
            self.step(self.timestep)
            
        while self.step(self.timestep) != -1:
            # Check for very close objects
            self.check_collisions()
            
            # Update LED states
            self.update_leds()

# Create and run the robot controller 
controller = EPuckController()
controller.run()