#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Float32
import csv
from datetime import datetime


def serial_reader():
    # Initialize ROS node
    rospy.init_node('serial_reader_node', anonymous=True)
    
    # Get serial port parameters from the parameter server
    port = rospy.get_param('~port', '/dev/ttyUSB0')  # Modify the serial port name according to your device
    baudrate = rospy.get_param('~baudrate', 115200)
    timeout = rospy.get_param('~timeout', 1)

    # # Create publishers for angle data and length data
    # bendlabs_pub = rospy.Publisher('bendlabs_data', Bendlabs, queue_size=10)

    # Open the serial port
    while True:
        try:
            ser = serial.Serial(port, baudrate, timeout=timeout)
            rospy.loginfo("Serial port %s opened", port)
            break
        except:
            rospy.logerr("BENDLABS: Error while opening serial port %s: %s", port)
            rospy.loginfo("BENDLABS: Retrying in 5 second...")
            rospy.sleep(5)
        
    # Set the loop rate for the ROS node
    while not rospy.is_shutdown():
        try:
            # Read data from the serial port
            data = ser.readline()

            # send data to the serial port
            ser.write(b'c0, 0\n')
            # if data:
            #     # Decode the received data
            #     decoded_data = data.decode('utf-8')

            #     # Split the decoded data by whitespace
            #     sets = decoded_data.split(',')

            #     # rospy.loginfo(sets)

            #     # if not csv_created:
            #     #     timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
            #     #     filename = f'data/bendlabs_{timestamp}.csv'
            #     #     csv_created = True
                
            #     # with open(filename, 'a', newline='') as file:
            #     #     writer = csv.writer(file)
            #     #     writer.writerow(sets[0:6])

            #     if len(sets) == 7:
            #         msg = Bendlabs()
            #         msg.header.stamp = rospy.Time.now()
            #         msg.sensor1_angle = float(sets[1])
            #         msg.sensor1_length = float(sets[2])
            #         msg.sensor2_angle = float(sets[4])
            #         msg.sensor2_length = float(sets[5])
            #         bendlabs_pub.publish(msg)

            # else:
            #     rospy.loginfo("No data received from serial port")
        except Exception as e:
            rospy.logwarn("Error while reading from serial port: %s", e)

    # Close the serial port
    ser.close()

if __name__ == '__main__':
    try:
        serial_reader()
    except rospy.ROSInterruptException:
        pass