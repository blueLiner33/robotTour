from machine import UART, Pin

# Initialize UART1 on the default pins for Pico (TX=Pin 4, RX=Pin 5)
uart1 = UART(1, baudrate=9600, tx=Pin(4), rx=Pin(5))

while True:
    if uart1.any():  # Check if there is data in the buffer
        data = uart1.read()  # Read all available data
        if data:
            print("Received:", data.decode('utf-8'))  # Decode if it's text data
