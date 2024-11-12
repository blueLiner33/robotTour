#(TX=Pin 4, RX=Pin 5)
uart1 = UART(1, baudrate=115200, tx=Pin(4), rx=Pin(5))

while True:
    if uart1.any():  
        data = uart1.read() 
        data = data.decode('utf-8')  
        
        
        data_list = data.split(",")  
        
        data_list = [int(i) for i in data_list]  
        print(data_list) 