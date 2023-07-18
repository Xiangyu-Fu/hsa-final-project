import serial # use pyserial for serial communication

import time

class serial_config:
    port = '/dev/ttyUSB0'
    baudrate = 115200
    bytesize = 8
    parity = 'N'
    # stopbits = 1
    

def main():
    # Open serial port

    ser = serial.Serial(serial_config.port, serial_config.baudrate)
    
    # ser.open()

    # Send a message
    while True:
        
        # read a input from user
        input_str = input("Enter a message: ")
        # print("You entered: " + input_str)
        
        input_str = f"{input_str}\n"
        
        # if ser.in_waiting == 0:
        #     print("In waiting")
        
        ser.write(input_str.encode())
        
        # send the message
        # ser.write(bytes(input_str, 'utf-8'))
        # Read a message
        print(ser.read(10).decode("utf-8"))
        time.sleep(1)
    

if __name__ == '__main__':
    
    main()
    
    

