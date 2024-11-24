from machine import Pin, Timer
import time

# Constants
PULSES_PER_REV = 240 
DEGREES_PER_COUNT = 360.0 / PULSES_PER_REV  

# Pins for encoders
ENCODER_A1 = 12  # Motor 1 encoder A pin
ENCODER_B1 = 11  # Motor 1 encoder B pin
ENCODER_A2 = 6   # Motor 2 encoder A pin
ENCODER_B2 = 7   # Motor 2 encoder B pin

# Globals to track encoder counts and last timestamps
encoder_count1 = 0
encoder_count2 = 0
last_time1 = 0
last_time2 = 0
rpm1 = 0
rpm2 = 0
degrees_one = 0
degrees_two = 0

# Interrupt service routines for encoders
def encoder_isr1(pin):
    global encoder_count1, last_time1
    encoder_count1 += 1
    last_time1 = time.ticks_us()

def encoder_isr2(pin):
    global encoder_count2, last_time2
    encoder_count2 += 1
    last_time2 = time.ticks_us()

# Calculate RPM and degrees
def calculate_metrics():
    global rpm1, rpm2, degrees_one, degrees_two, last_time1, last_time2

    current_time1 = time.ticks_us()
    current_time2 = time.ticks_us()

    # Calculate RPM for motor 1
    if encoder_count1 > 0 and current_time1 != last_time1:
        time_diff1 = time.ticks_diff(current_time1, last_time1)
        rpm1 = (60_000_000.0 / (time_diff1 * PULSES_PER_REV)) if time_diff1 > 0 else 0
    else:
        rpm1 = 0

    # Calculate RPM for motor 2
    if encoder_count2 > 0 and current_time2 != last_time2:
        time_diff2 = time.ticks_diff(current_time2, last_time2)
        rpm2 = (60_000_000.0 / (time_diff2 * PULSES_PER_REV)) if time_diff2 > 0 else 0
    else:
        rpm2 = 0

    # Calculate degrees and wrap around 360
    degrees_one = (encoder_count1 * DEGREES_PER_COUNT) % 360 - 6
    degrees_two = (encoder_count2 * DEGREES_PER_COUNT) % 360 - 6 

# Setup pins and interrupts
encoder_pin_a1 = Pin(ENCODER_A1, Pin.IN, Pin.PULL_UP)
encoder_pin_b1 = Pin(ENCODER_B1, Pin.IN, Pin.PULL_UP)
encoder_pin_a2 = Pin(ENCODER_A2, Pin.IN, Pin.PULL_UP)
encoder_pin_b2 = Pin(ENCODER_B2, Pin.IN, Pin.PULL_UP)

encoder_pin_a1.irq(trigger=Pin.IRQ_RISING, handler=encoder_isr1)
encoder_pin_a2.irq(trigger=Pin.IRQ_RISING, handler=encoder_isr2)

# Main loop
while True:
    calculate_metrics()
    print(f"Motor 1 - RPM: {rpm1:.2f}, Degrees: {encoder_count1:.2f}")
    print(f"Motor 2 - RPM: {rpm2:.2f}, Degrees: {degrees_two:.2f}")
    time.sleep(0.2)  # Adjust the delay for desired refresh rate

