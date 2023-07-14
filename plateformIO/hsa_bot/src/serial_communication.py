import serial # use pyserial for serial communication

import time

class serial_config:
    port = '/dev/ttyUSB0'
    baudrate = 115200
    bytesize = 8
    parity = 'N'
    stopbits = 0
    

def main():
    # Open serial port
    ser = serial.Serial(serial_config.port, serial_config.baudrate, timeout=1)

    # Send a message
    ser.write(b'Hello world!')
    
    while True:
        
        # read a input from user
        input_str = input("Enter a message: ")
        print("You entered: " + input_str)
        
        # send the message
        ser.write(input_str.encode())
        # Read a message
        print(ser.read(10))
    

if __name__ == '__main__':
    
    main()
    
    

