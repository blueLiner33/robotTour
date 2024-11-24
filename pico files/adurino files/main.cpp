import time
from struct import unpack_from
from machine import UART, Pin, Timer


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

# Main loop
while True:
    try:
        yaw, pitch, roll, x_accel, y_accel, z_accel = rvc.heading
        print("Yaw: %2.2f Pitch: %2.2f Roll: %2.2f Degrees" % (yaw, pitch, roll))
        print("Acceleration X: %2.2f Y: %2.2f Z: %2.2f m/s^2" % (x_accel, y_accel, z_accel))
        print("")
    except RVCReadTimeoutError as e:
        print("Timeout reading RVC heading:", e)
    time.sleep(0.1)

