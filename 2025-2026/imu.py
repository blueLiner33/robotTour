import time
from struct import unpack_from
from machine import UART, Pin, Timer
from config import uart_tx_pin, uart_rx_pin

uart0 = UART(0, baudrate=115200, tx=Pin(uart_tx_pin), rx=Pin(uart_rx_pin)) 

class RVCReadTimeoutError(Exception):
    """Raised if a UART-RVC message cannot be read before the given timeout."""
class BNO08x_RVC:
    def __init__(self, uart: UART, timeout: float = 1.0) -> None:
        self._uart = uart
        self._debug = True
        self._read_timeout = int(timeout * 1000)  
        self._last_time = None 
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
        start_time = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), start_time) < self._read_timeout:
            data = self._uart.read(2)
            if not data or len(data) < 2:
                continue
            if data[0] == 0xAA and data[1] == 0xAA:
                msg = self._uart.read(17)
                if not msg or len(msg) < 17:
                    continue
                heading = self._parse_frame(msg)
                if heading is None:
                    continue

                
                now = time.ticks_ms()
                if self._last_time is None:
                    dt = 0
                else:
                    dt = time.ticks_diff(now, self._last_time) / 1000  # seconds
                self._last_time = now

                return (heading, dt)
            
        raise RVCReadTimeoutError("Unable to read RVC heading message")
    
rvc = BNO08x_RVC(uart0)

