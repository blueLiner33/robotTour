import time
from struct import unpack_from
from machine import UART, Pin, Timer

#********************************encoders*************************************************
# Constants
PULSES_PER_REV = 240 
DEGREES_PER_COUNT = 360.0 / PULSES_PER_REV  

# Pins for encoders
ENCODER_A1 = 0  # Motor 1 encoder A pin
ENCODER_B1 = 1  # Motor 1 encoder B pin
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
    degrees_one = (encoder_count1 * DEGREES_PER_COUNT) % 360 
    degrees_two = (encoder_count2 * DEGREES_PER_COUNT) % 360 

# Setup pins and interrupts
encoder_pin_a1 = Pin(ENCODER_A1, Pin.IN, Pin.PULL_UP)
encoder_pin_b1 = Pin(ENCODER_B1, Pin.IN, Pin.PULL_UP)
encoder_pin_a2 = Pin(ENCODER_A2, Pin.IN, Pin.PULL_UP)
encoder_pin_b2 = Pin(ENCODER_B2, Pin.IN, Pin.PULL_UP)

encoder_pin_a1.irq(trigger=Pin.IRQ_RISING, handler=encoder_isr1)
encoder_pin_a2.irq(trigger=Pin.IRQ_RISING, handler=encoder_isr2)

#**************************************************BNO08x sensor*********************************************************

class RVCReadTimeoutError(Exception):
    """Raised if a UART-RVC message cannot be read before the given timeout."""


class BNO08x_RVC:
    def __init__(self, uart: UART, timeout: float = 1.0) -> None:
        self._uart = uart
        self._debug = True
        self._read_timeout = int(timeout * 1000)  # Convert timeout to milliseconds

    @staticmethod
    def _parse_frame(frame: bytes):
        """Parse a frame of data from the BNO08x."""
        try:
            heading_frame = unpack_from("<BhhhhhhBBBB", frame)
            (
                _index,
                yaw,
                pitch,
                roll,
                x_accel,
                y_accel,
                z_accel,
                _res1,
                _res2,
                _res3,
                _checksum,
            ) = heading_frame
            checksum_calc = sum(frame[0:16]) % 256
            if checksum_calc != _checksum:
                return None
            yaw *= 0.01
            pitch *= 0.01
            roll *= 0.01
            x_accel *= 0.0098067
            y_accel *= 0.0098067
            z_accel *= 0.0098067
            return (yaw, pitch, roll, x_accel, y_accel, z_accel)
        except Exception as e:
            if self._debug:
                print("Error parsing frame:", e)
            return None

    @property
    def heading(self):
        """Fetch the current heading."""
        start_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start_time) < self._read_timeout:
            data = self._uart.read(2)
            if data is None or len(data) < 2:  # Ensure data is valid
                continue
            if data[0] == 0xAA and data[1] == 0xAA:
                msg = self._uart.read(17)
                if msg is None or len(msg) < 17:  # Ensure full frame is read
                    continue
                heading = self._parse_frame(msg)
                if heading is None:
                    continue
                return heading
        raise RVCReadTimeoutError("Unable to read RVC heading message")


# Initialize UART(0) with TX and RX pins
uart0 = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))

# Create an instance of BNO08x_RVC with uart0
rvc = BNO08x_RVC(uart0)


def get_data():
    try:
        average_data = []
        calculate_metrics()
        yaw, x_accel, y_accel, z_accel = rvc.heading
        average_data.append ([rpm1,degrees_one,rpm2,degrees_two,yaw,x_accel,y_accel,z_accel])
        if len(average_data) > 5:  # returns average of 5 values
            column_average = [sum(sub_list) / len(sub_list) for sub_list in zip(*average_data)]
            rounded_average = [round(elem, 2) for elem in column_average]
            average_data.pop(0)
            return(rounded_average)
        else:
            pass
    except RVCReadTimeoutError as e:
        raise e