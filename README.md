# Ball Position Control System using Raspberry Pi Pico and MicroPython



## Overview



This project demonstrates the implementation of a ball position control system using a Raspberry Pi Pico, a Sharp IR sensor, and a fan. The control algorithm used is a PID controller, which maintains the ball at a desired height within a tube by adjusting the fan speed.



## Hardware Requirements



- Raspberry Pi Pico

- Sharp IR Distance Sensor

- Fan connected via PWM

- Wires and a tube (for the ball)



## MicroPython Code



### Code Explanation



The provided MicroPython code utilizes the following concepts to achieve ball position control:



1. **PWM Setup:**

   Initializes PWM on GPIO 17 to control the fan speed.

   ```python

   fan_pwm = machine.PWM(machine.Pin(17))

   fan_pwm.freq(25000)  # Set frequency to 25kHz

   ```



2. **ADC Setup:**

   Initializes ADC on GPIO 26 to read distance values from the Sharp IR sensor.

   ```python

   adc = machine.ADC(machine.Pin(26))

   ```



3. **Calibration Values:**

   Define calibration values for mapping raw ADC readings to distances in centimeters.

   ```python

   min_adc = 9000    # Raw ADC value at tube bottom

   max_adc = 30000   # Raw ADC value at tube top

   tube_height = 22  # Tube height in centimeters

   ```



4. **Mapping Functions:**

   - `map_adc_to_distance`: Converts raw ADC values to a distance in centimeters.

   - `map_target_to_distance`: Converts a target percentage to an actual distance.

   ```python

   def map_adc_to_distance(adc_value):

       ...

   ```



5. **PID Controller Class:**

   Implements a PID controller for controlling the fan speed to maintain the ball at the desired height.

   ```python

   class PID:

       ...

   ```



6. **Reading Distance:**

   Reads and maps the distance from the IR sensor, printing debug information.

   ```python

   def read_distance():

       ...

   ```



7. **Setting Fan Speed:**

   Sets the PWM duty cycle to control the fan speed, ensuring it's within a valid range (0-100%).

   ```python

   def set_fan_speed(duty_cycle):

       ...

   ```



8. **Control Ball Position:**

   Uses the PID controller to adjust the fan speed and maintain the ball at the desired height.

   ```python

   def control_ball():

       ...

   ```



9. **Sweeping Mode:**

   Implements a sweeping mode to move the ball up and down within the tube, adjusting the target distance and fan speed accordingly.

   ```python

   def sweep_ball(min_dist, max_dist):

       ...

   ```



10. **Main Loop:**

    Runs continuously, either in sweeping mode or maintaining a target position based on user-defined parameters.

    ```python

    while True:

        ...

    ```



## Usage Instructions



1. **Connect Hardware:**

   - Connect the fan to GPIO 17.

   - Connect the Sharp IR sensor to GPIO 26.

   - Ensure all connections are secure.



2. **Load the Code:**

   - Upload the code to your Raspberry Pi Pico.



3. **Run the Code:**

   - Connect your Raspberry Pi Pico to a power source.

   - The code will start running automatically, controlling the ball's position within the tube.



4. **Monitor Output:**

   - Use a serial monitor to observe debug outputs and ensure everything is functioning as expected.



## Customization



- Adjust the PID parameters (`kp`, `ki`, `kd`) for optimal performance.

- Update the calibration values (`min_adc`, `max_adc`) specific to your setup.

- Modify target distance percentage or enable/disable sweep mode as desired.
