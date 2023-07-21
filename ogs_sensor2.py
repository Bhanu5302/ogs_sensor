#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Int32, Bool,String
import serial
import time
from threading import Thread

moco_target = 0
moco_current_pos = 0
ogs2_status = False
teach_cmd = "none"
moco_mode_status = 'manual'
semi_auto_status = False
timer = time.time()



def moco_target_pose_callback(msg: Int32):
    global moco_target
    moco_target = msg.data


def moco_current_pose_callback(msg: Int32):
    global moco_current_pos
    moco_current_pos = msg.data


def ogs2_teach_callback(msg: String):
    global teach_cmd
    teach_cmd = msg.data


def moco_mode_status_callback(msg: String):
    global moco_mode_status
    moco_mode_status = msg.data

def ogs2_read():
    trace_data2 = bytearray(b'\x21\x00\xCF\x00\x00\xEE\x00\x00')  # VALID SUB PIXEL TRACE EDGES READ IN HEX
    ComPort2.write(trace_data2)
    dataIn2 = ComPort2.read()
    dataIn2 += ComPort2.read(ComPort2.inWaiting())
    read_left2 = (dataIn2[5] + (dataIn2[6] << 8)) / 10  # Left edge Trace subpixel
    read_right2 = (dataIn2[7] + (dataIn2[8] << 8)) / 10  # Right edge Trace subpixel
    if ComPort2.isOpen():
        if read_left2 == 0 or read_right2 == 0:
            trace_data2 = bytearray(b'\x21\x00\xD5\x00\x00\xF4\x00\x00')  # VALID SUB PIXEL TRACE EDGES READ IN HEX
            ComPort2.write(trace_data2)
            # print(trace_data2)  # print the data
            dataIn2 = ComPort2.read()
            dataIn2 += ComPort2.read(ComPort2.inWaiting())
            # dataIn2 = ComPort2.read(24)
            # print(dataIn2)
            read_left2 = (dataIn2[5] + (dataIn2[6] << 8)) / 10  # Left edge Trace subpixel
            read_right2 = (dataIn2[7] + (dataIn2[8] << 8)) / 10  # Right edge Trace subpixel

        right2_msg = Float64()
        left2_msg = Float64()
        ogs2_status = True

        right2_msg.data = read_right2
        left2_msg.data = read_left2

        ogs2_status_msg = Bool()
        ogs2_status_msg.data = ogs2_status

        right2_publisher.publish(right2_msg)
        left2_publisher.publish(left2_msg)

        ogs2_status_publisher.publish(ogs2_status_msg)
    else:
        ComPort2.open()


if __name__ == "__main__":
    ComPort2 = serial.Serial('/dev/ogs2')  # open COM8
    ComPort2.baudrate = 115200  # set Baud rate to 115200
    ComPort2.bytesize = 8  # Number of data bits = 8
    ComPort2.parity = 'O'  # Odd parity
    ComPort2.stopbits = 1  # Number of Stop bits = 1
    ComPort2.timeout = 4

    rospy.init_node("ogs_sensor2_node")
    rospy.loginfo("ogs_sensor - 2_node node has been started.")

    moco_target_pose_subscriber = rospy.Subscriber("/moco/rf_id_target", Int32, callback=moco_target_pose_callback,
                                                   queue_size=1)
    current_position_subscriber = rospy.Subscriber("/moco/current_pos", Int32, callback=moco_current_pose_callback,
                                                   queue_size=1)
    ogs2_teach_subscriber = rospy.Subscriber("/ogs2_sensor/teach", String, callback=ogs2_teach_callback,
                                             queue_size=1)

    moco_mode_status_subscriber = rospy.Subscriber("/moco/mode", String, callback=moco_mode_status_callback, queue_size=1)

    right2_publisher = rospy.Publisher("/ogs_sensor2/right", Float64, queue_size=5)
    left2_publisher = rospy.Publisher("/ogs_sensor2/left", Float64, queue_size=5)
    ogs2_status_publisher = rospy.Publisher("/ogs2_sensor/status", Bool, queue_size=0)
    ogs2_teach_response_publisher = rospy.Publisher("/ogs2_sensor/teach_status", String, queue_size=0)
    rate = rospy.Rate(60)
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():

        if ComPort2.isOpen() and (moco_mode_status == 'auto' or moco_mode_status == 'semi-auto'):
            try:
                ogs2_read() # calling ogs2_reading function

            except Exception as e:
                print(e)
                print("OGS sensor-2 Read failed")
                right2_msg = Float64()
                left2_msg = Float64()
                ogs2_status_msg = Bool()
                ComPort2.close()
                right2_msg.data = 0
                left2_msg.data = 0
                ogs2_status = False

                ogs2_status_msg.data = ogs2_status

                right2_publisher.publish(right2_msg)
                left2_publisher.publish(left2_msg)

                right2_publisher.publish(0)
                left2_publisher.publish(0)
                ogs2_status_publisher.publish(ogs2_status_msg)

        elif moco_mode_status == 'manual':
            if teach_cmd == "width":
                try:
                    teach_width = bytearray(
                        b'\x22\x01\x02\x00\x00\xC2\xE3\x00')  # VALID SUB PIXEL TRACE EDGES READ IN HEX
                    ComPort2.write(teach_width)
                    teach_width_read = ComPort2.read()
                    teach_width_read += ComPort2.read(ComPort2.inWaiting())
                    # print(teach_width_read)
                    width_response = teach_width_read[5]  # teach width read
                    if width_response == 0x2a:
                        ogs2_teach_response_publisher.publish("width")
                    else:
                        ogs2_teach_response_publisher.publish("fail")

                except:
                    print("Trace width teach failed")
                    ogs2_teach_response_publisher.publish("fail")

            elif teach_cmd == "contrast":
                try:
                    teach_contrast = bytearray(
                        b'\x22\x01\x02\x00\x00\xC3\xE2\x00')  # VALID SUB PIXEL TRACE EDGES READ IN HEX
                    ComPort2.write(teach_contrast)
                    teach_contrast_read = ComPort2.read()
                    teach_contrast_read += ComPort2.read(ComPort2.inWaiting())
                    # print(teach_width_read)
                    contrast_response = teach_contrast_read[5]  # teach width read
                    if contrast_response == 0x2a:
                        ogs2_teach_response_publisher.publish("contrast")
                    else:
                        ogs2_teach_response_publisher.publish("fail")

                except:
                    print("Trace contrast teach failed")
                    ogs2_teach_response_publisher.publish("fail")

            elif teach_cmd == "amplitude":
                try:
                    teach_amplitude = bytearray(
                        b'\x22\x01\x02\x00\x00\xC4\xE5\x00')  # VALID SUB PIXEL TRACE EDGES READ IN HEX
                    ComPort2.write(teach_amplitude)
                    teach_amplitude_read = ComPort2.read()
                    teach_amplitude_read += ComPort2.read(ComPort2.inWaiting())
                    # print(teach_width_read)
                    amplitude_response = teach_amplitude_read[5]  # teach width read
                    if amplitude_response == 0x2a:
                        ogs2_teach_response_publisher.publish("amplitude")
                    else:
                        ogs2_teach_response_publisher.publish("fail")

                except:
                    print("Trace amplitude teach failed")
                    ogs2_teach_response_publisher.publish("fail")

            elif teach_cmd == "all":
                try:
                    teach_all = bytearray(
                        b'\x22\x01\x02\x00\x00\xC0\xE1\x00')  # VALID SUB PIXEL TRACE EDGES READ IN HEX
                    ComPort2.write(teach_all)
                    teach_all_read = ComPort2.read()
                    teach_all_read += ComPort2.read(ComPort2.inWaiting())
                    print(teach_all_read)
                    all_response = teach_all_read[5]  # teach width read
                    if all_response == 0x2a:
                        ogs2_teach_response_publisher.publish("all")
                    else:
                        ogs2_teach_response_publisher.publish("fail")

                except:
                    print("OGS-2 teach all failed")
                    ogs2_teach_response_publisher.publish("fail")

            elif teach_cmd == "ledon":
                try:
                    led_on = bytearray(
                        b'\x22\x01\x02\x00\x00\xB0\x91\x00')  # VALID SUB PIXEL TRACE EDGES READ IN HEX
                    ComPort2.write(led_on)
                    led_on_read = ComPort2.read()
                    led_on_read += ComPort2.read(ComPort2.inWaiting())
                    # print(teach_width_read)
                    led_on_response = led_on_read[5]  # teach width read
                    if led_on_response == 0x2a:
                        ogs2_teach_response_publisher.publish("ledon")
                    else:
                        ogs2_teach_response_publisher.publish("fail")

                except:
                    print("OGS-2 led on failed")
                    ogs2_teach_response_publisher.publish("fail")

            elif teach_cmd == "ledoff":
                try:
                    led_off = bytearray(
                        b'\x22\x01\x02\x00\x00\xB1\x90\x00')  # VALID SUB PIXEL TRACE EDGES READ IN HEX
                    ComPort2.write(led_off)
                    led_off_read = ComPort2.read()
                    led_off_read += ComPort2.read(ComPort2.inWaiting())
                    # print(teach_width_read)
                    led_off_response = led_off_read[5]  # teach width read
                    if led_off_response == 0x2a:
                        ogs2_teach_response_publisher.publish("ledoff")
                    else:
                        ogs2_teach_response_publisher.publish("fail")

                except:
                    print("OGS-2 led off failed")
                    ogs2_teach_response_publisher.publish("fail")

            elif teach_cmd == "light":
                try:
                    trace_light = bytearray(
                        b'\x22\x01\x02\x00\x00\xD5\xF4\x00')  # VALID SUB PIXEL TRACE EDGES READ IN HEX
                    ComPort2.write(trace_light)
                    trace_light_read = ComPort2.read()
                    trace_light_read += ComPort2.read(ComPort2.inWaiting())
                    print(trace_light_read)
                    trace_light_response = trace_light_read[5]

                    if trace_light_response == 0x2a:
                        ogs2_teach_response_publisher.publish("light")
                    else:
                        ogs2_teach_response_publisher.publish("fail")

                except Exception as e:
                    print(e)
                    ogs2_teach_response_publisher.publish("fail")

            elif teach_cmd == "dark":
                try:
                    trace_dark = bytearray(
                        b'\x22\x01\x02\x00\x00\xD4\xF5\x00')  # VALID SUB PIXEL TRACE EDGES READ IN HEX
                    ComPort2.write(trace_dark)
                    trace_dark_read = ComPort2.read()
                    trace_dark_read += ComPort2.read(ComPort2.inWaiting())
                    print(trace_dark_read)

                    trace_dark_response = trace_dark_read[5]  # trace dark
                    if trace_dark_response == 0x2a:
                        ogs2_teach_response_publisher.publish("dark")
                    else:
                        ogs2_teach_response_publisher.publish("fail")
                except Exception as e:
                    print(e)
                    ogs2_teach_response_publisher.publish("fail")
        elif not ComPort2:
            print('trying to open port')
            ComPort2.open()

        else:
            ComPort2.close()
            print(" OGS sensor 2 Port is not opened")
            time.sleep(2)
            ComPort2.open()
            print("Opening OGS2 COM port")
