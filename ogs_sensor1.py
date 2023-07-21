#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64, Int32, Bool, String, Float64MultiArray
import serial
import time
import OGS

moco_target = 0
moco_current_pos = 0
ogs1_status = False
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


def ogs1_teach_callback(msg: String):
    global teach_cmd
    teach_cmd = msg.data


def moco_mode_status_callback(msg: String):
    global moco_mode_status
    moco_mode_status = msg.data


def ogs1_read():
    global edge_data
    read_left,read_right=OGS_cx.edge_read(bytearray(b'\x11\x00\xCF\x00\x00\xDE\x00\x00'))

    if OGS1.isOpen():
        if read_left == 0 or read_right == 0:
            read_left,read_right=OGS_cx.edge_read(bytearray(b'\x11\x00\xD5\x00\x00\xC4\x00\x00'))
            ogs1_status = True
            ogs1_status_msg = Bool()
            ogs1_status_msg.data = ogs1_status
            #-------Publish OGS-edge data in Float64MultiArray--------
            edge_data=Float64MultiArray()
            edge_data = [read_left, read_right]
            ogs1_edges_publisher.publish(edge_data)
            #-------------------------------------------------------

            ogs1_status_publisher.publish(ogs1_status_msg)  #Publish OGS status
            # rate.sleep()

    else:
        OGS1.open()


if __name__ == "__main__":
    global teach_cmd,edge_data
    
    OGS_cx=OGS.OGS(port='/dev/ogs1')
    OGS1=OGS.OGS.connect(OGS_cx) # open COM
    print(OGS1)
    rospy.init_node("ogs_sensor_1_node")
    rospy.loginfo("ogs_sensor_1 _node node has been started.")
    #---------------------------------------ROS Subscribers------------------------------------------------
    current_position_subscriber = rospy.Subscriber("/moco/current_pos", Int32, callback=moco_current_pose_callback,
                                                   queue_size=1)
    moco_mode_status_subscriber = rospy.Subscriber("/moco/mode", String, callback=moco_mode_status_callback,
                                                   queue_size=1)
    ogs1_teach_subscriber = rospy.Subscriber("/ogs1_sensor/teach", String, callback=ogs1_teach_callback,
                                             queue_size=1)
    #-------------------------------------------------------------------------------------------------------

    #--------------------------------------ROS Publishers---------------------------------------------------
    ogs1_status_publisher = rospy.Publisher("/ogs1_sensor/status", Bool, queue_size=0)
    ogs1_edges_publisher = rospy.Publisher("/ogs_sensor/edges", Float64MultiArray, queue_size=5)
    ogs1_teach_response_publisher = rospy.Publisher("/ogs1_sensor/teach_status", String, queue_size=0)
    #-------------------------------------------------------------------------------------------------------
    rate = rospy.Rate(60)

    while not rospy.is_shutdown():
        if OGS1.isOpen() and (moco_mode_status == 'auto' or moco_mode_status == 'semi-auto'):
            try:
                ogs1_read()  # calling ogs1_reading function
            except Exception as e:
                print(e)
                print("OGS sensor-1 read failed")
                OGS1.close()
                OGS1.open()

                # -------Publish OGS-edge data in Float64MultiArray--------
                edge_data = [0.0, 0.0]
                ogs1_edges_publisher.publish(edge_data)
                #----------------------------------------------------------
                ogs1_status_publisher.publish(False)

        elif moco_mode_status == 'manual':
            if teach_cmd == "width":
                try:
                    print("ogs1- width teaching")
                    width_response=OGS_cx.teach_ogs(cmd=bytearray(b'\x12\x01\x02\x00\x00\xC2\xD3\x00'))
                    if width_response == 0x1a:
                        ogs1_teach_response_publisher.publish("width")
                    else:
                        ogs1_teach_response_publisher.publish("fail")
                except:
                    print("Trace width teach failed")
                    ogs1_teach_response_publisher.publish("fail")

            elif teach_cmd == "contrast":
                try:
                    print("ogs1- contrast teaching")
                    contrast_response=OGS_cx.teach_ogs(cmd=bytearray(b'\x12\x01\x02\x00\x00\xC3\xD2\x00'))
                    if contrast_response == 0x1a:
                        ogs1_teach_response_publisher.publish("contrast")
                    else:
                        ogs1_teach_response_publisher.publish("fail")
                except:
                    print("Trace contrast teach failed")
                    ogs1_teach_response_publisher.publish("fail")

            elif teach_cmd == "amplitude":
                try:
                    print("ogs1- teaching amplitude")
                    amplitude_response=OGS_cx.teach_ogs(cmd=bytearray(b'\x12\x01\x02\x00\x00\xC4\xD5\x00'))
                    if amplitude_response == 0x1a:
                        ogs1_teach_response_publisher.publish("amplitude")
                    else:
                        ogs1_teach_response_publisher.publish("fail")

                except:
                    print("Trace amplitude teach failed")
                    ogs1_teach_response_publisher.publish("fail")

            elif teach_cmd == "all":
                try:
                    print("ogs1-teaching all")
                    all_response=OGS_cx.teach_ogs(cmd=bytearray(b'\x12\x01\x02\x00\x00\xC0\xD1\x00'))
                    if all_response == 0x1a:
                        ogs1_teach_response_publisher.publish("all")
                    else:
                        ogs1_teach_response_publisher.publish("fail")

                except:
                    print("OGS-1 teach all failed")
                    ogs1_teach_response_publisher.publish("fail")

            elif teach_cmd == "ledon":
                try:
                    print("OGS1-LED on")
                    led_on_response=OGS_cx.teach_ogs(cmd=bytearray(b'\x12\x01\x02\x00\x00\xB0\xA1\x00'))
                    if led_on_response == 0x1a:
                        ogs1_teach_response_publisher.publish("ledon")
                    else:
                        ogs1_teach_response_publisher.publish("fail")


                except:
                    print("OGS-1 led on failed")
                    ogs1_teach_response_publisher.publish("fail")

            elif teach_cmd == "ledoff":
                try:
                    print("OGS1-LED off")
                    led_off_response=OGS_cx.teach_ogs(cmd=bytearray(b'\x12\x01\x02\x00\x00\xB1\xA0\x00'))
                    if led_off_response == 0x1a:
                        ogs1_teach_response_publisher.publish("ledoff")
                    else:
                        ogs1_teach_response_publisher.publish("fail")
                except:
                    print("OGS-1 led off failed")
                    ogs1_teach_response_publisher.publish("fail")

            elif teach_cmd == "light":
                try:
                    print("trace light")
                    trace_light_response=OGS_cx.teach_ogs(cmd=bytearray(b'\x12\x01\x02\x00\x00\xD5\xC4\x00'))
                    if trace_light_response == 0x1a:
                        ogs1_teach_response_publisher.publish("light")
                    else:
                        ogs1_teach_response_publisher.publish("fail")
                except Exception as e:
                    print(e)
                    ogs1_teach_response_publisher.publish("fail")

            elif teach_cmd == "dark":
                try:
                    print("trace dark")
                    trace_dark_response=OGS_cx.teach_ogs(cmd=bytearray(b'\x12\x01\x02\x00\x00\xD4\xC5\x00'))
                    if trace_dark_response == 0x1a:
                        ogs1_teach_response_publisher.publish("dark")
                    else:
                        ogs1_teach_response_publisher.publish("fail")
                except Exception as e:
                    print(e)
                    ogs1_teach_response_publisher.publish("fail")
        elif not OGS1:
            print('trying to open port')
            OGS1.open()
        else:
            OGS1.close()
            print(" OGS Sensor-1 Port is not opened")
            time.sleep(2)
            OGS1.open()
            print("Opening OGS1 COM port")
