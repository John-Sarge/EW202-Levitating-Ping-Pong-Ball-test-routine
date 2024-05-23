# Written by John Seargeant, WRC TSD

import machine
import time

# Setup for PWM on GP17
fan_pwm = machine.PWM(machine.Pin(17))
fan_pwm.freq(25000)  # Set frequency to 25kHz

# Setup for the Sharp IR sensor on GP26
adc = machine.ADC(machine.Pin(26))

# Calibration values (replace these with your actual values)
min_adc = 9000    # Raw ADC value at tube bottom
max_adc = 30000   # Raw ADC value at tube top
tube_height = 22  # Tube height in centimeters

def map_adc_to_distance(adc_value):
    """ Map ADC value to distance in cm based on calibration. """
    if adc_value <= min_adc:
        return 0
    elif adc_value >= max_adc:
        return tube_height
    else:
        distance = ((adc_value - min_adc) * tube_height / (max_adc - min_adc))
        return max(0, min(tube_height, distance))

def map_target_to_distance(min_dist, max_dist, percentage):
    """ Map target percentage to actual distance. """
    return min_dist + ((max_dist - min_dist) * (percentage / 100.0))

# PID Controller Class
class PID:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.previous_time = time.ticks_ms()
        
    def compute(self, process_variable):
        current_time = time.ticks_ms()
        delta_time = time.ticks_diff(current_time, self.previous_time) / 1000.0
        
        error = self.setpoint - process_variable
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(-50, min(50, output))  # Clamp output to reasonable range

        self.previous_error = error
        self.previous_time = current_time
        
        return output

def read_distance():
    """ Read distance from the IR sensor. """
    adc_value = adc.read_u16()
    distance = map_adc_to_distance(adc_value)
    print(f"ADC Value: {adc_value}, Mapped Distance: {distance:.2f}")  # Debug output
    return distance

def set_fan_speed(duty_cycle):
    """ Set the fan speed via PWM duty cycle. """
    duty_cycle = max(0, min(100, duty_cycle))  # Ensure duty cycle is within [0, 100]
    fan_pwm.duty_u16(int(duty_cycle * 65535 / 100))
    print(f"Fan PWM Duty Cycle: {duty_cycle:.2f}%")  # Debug output

def control_ball():
    """ Control the ball position using PID controller. """
    current_distance = read_distance()
    pid_output = pid.compute(current_distance)
    
    duty_cycle = 50 + pid_output  # Adjust based on PID output
    print(f"PID Output: {pid_output:.2f}, Calculated Duty Cycle: {duty_cycle:.2f}%")  # Debug output
    
    set_fan_speed(duty_cycle)
    return current_distance

def sweep_ball(min_dist, max_dist):
    """ Sweep the ball up and down. """
    global sweep_direction
    global target_distance
    
    current_distance = read_distance()
    if current_distance >= max_dist:
        sweep_direction = -1
    elif current_distance <= min_dist:
        sweep_direction = 1
        
    step_size = 0.25  # Smaller step size for smoother sweeping
    target_distance += (sweep_direction * step_size)
    
    # Clamp target_distance within bounds
    target_distance = max(min_dist, min(max_dist, target_distance))
    print(f"Sweep Direction: {sweep_direction}, New Target Distance: {target_distance:.2f}")  # Debug output

    pid.setpoint = target_distance
    control_ball()
    return target_distance

# Initial parameters
min_distance = 0
max_distance = tube_height  # Ensuring this is consistent with tube height
target_percentage = 50  # Set this to control the initial position as a percentage of tube height
sweep_mode = True
sweep_direction = 1

target_distance = map_target_to_distance(min_distance, max_distance, target_percentage)
pid = PID(kp=0.8, ki=0.2, kd=0.1, setpoint=target_distance)  # Adjusted PID parameters

while True:
    if sweep_mode:
        # Sweep mode
        print("Sweeping ball...")
        current_distance = sweep_ball(min_distance, max_distance)
    else:
        # Normal mode
        target_distance = map_target_to_distance(min_distance, max_distance, target_percentage)
        pid.setpoint = target_distance
        current_distance = control_ball()
        print(f"Current Distance: {current_distance:.2f}, Target Distance: {target_distance:.2f}")
    
    time.sleep(0.1)  # Add a delay to avoid excessive CPU usage
