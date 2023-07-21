#!/usr/bin/env python3
# Optimized OGS sensor code for MOCO AGV
import rospy
from std_msgs.msg import Float64, Int32, Bool, String, Float64MultiArray
import time
from OGS import *

moco_target = moco_current_pos = 0
ogs1_status = ogs2_status = teach_result = semi_auto_status = False
teach_cmd = "none"
moco_mode_status = 'manual'
timer = time.time()


# Add other OGS commands here...
# ----------------------------------OGS1 sensor edges read block---------------------------------
def ogs_read(ogs, ogs_valid_pixel_read, ogs_invalid_pixel_read, ogs_edges_publisher, ogs_status_publisher):
    ogs_status_msg = Bool()
    edge_data = Float64MultiArray()
    try:
        read_left, read_right = ogs.edge_read(ogs_valid_pixel_read)
        if read_left == 0 or read_right == 0:
            read_left, read_right = ogs.edge_read(ogs_invalid_pixel_read)
        ogs_status_msg.data = True

        # Publish OGS-edge data in Float64MultiArray
        edge_data.data = [read_left, read_right]
        ogs_edges_publisher.publish(edge_data)

        ogs_status_publisher.publish(ogs_status_msg)  # Publish OGS status
    except:
        ogs_status_msg.data = False
        ogs_status_publisher.publish(ogs_status_msg)  # Publish OGS status


# ------------------------------------------------------------------------------------------------

def moco_current_pose_callback(msg: Int32):
    global moco_current_pos
    moco_current_pos = msg.data


def moco_mode_status_callback(msg: String):
    global moco_mode_status
    moco_mode_status = msg.data


# ------------OGS-1 and OGS-2 sensor teaching block START-------------------
def teach_ogs(teach_cmd, OGS_cx):
    try:
        response = OGS_cx.teach_ogs(cmd=teach_cmd)
        if response == 0x1a or 0x2a:
            return True
    except Exception as e:
        print(e)
    return False


def ogs_teach_callback(msg: String):
    global teach_result
    teach_cmd = None
    get_teach_cmd = {"width1": OGS1_CMD_WIDTH, "contrast1": OGS1_CMD_CONTRAST, "amplitude1": OGS1_CMD_AMPLITUDE,
                     "all1": OGS1_CMD_ALL, "ledon1": OGS1_CMD_LED_ON, "ledoff1": OGS1_CMD_LED_OFF,
                     "light1": OGS1_CMD_LIGHT, "dark1": OGS1_CMD_DARK,
                     "width2": OGS2_CMD_WIDTH, "contrast2": OGS2_CMD_CONTRAST, "amplitude2": OGS2_CMD_AMPLITUDE,
                     "all2": OGS2_CMD_ALL, "ledon2": OGS2_CMD_LED_ON, "ledoff2": OGS2_CMD_LED_OFF,
                     "light2": OGS2_CMD_LIGHT, "dark2": OGS2_CMD_DARK
                     }

    teach_cmd = get_teach_cmd[msg.data]
    if "1" in msg.data:
        teach_result = teach_ogs(teach_cmd, OGS1)
    elif "2" in msg.data:
        teach_result = teach_ogs(teach_cmd, OGS2)
    if teach_result:
        ogs_teach_response_publisher.publish(msg.data)
    else:
        ogs_teach_response_publisher.publish("fail")


# ----------------OGS-1 and OGS-2 sensor teaching block END-------------------

if __name__ == "__main__":
    OGS_cx1 = OGS(port='/dev/ogs1')
    OGS_cx2 = OGS(port='/dev/ogs2')
    OGS1 = OGS.connect(OGS_cx1)
    OGS2 = OGS.connect(OGS_cx2)

    # -------------------------------------ROS Initialization---------------------------------------------------
    rospy.init_node("ogs_sensor_1&2_node")
    rospy.loginfo("ogs_sensor_1&2 _node has been started.")

    current_position_subscriber = rospy.Subscriber("/moco/current_pos", Int32, callback=moco_current_pose_callback,
                                                   queue_size=1)
    moco_mode_status_subscriber = rospy.Subscriber("/moco/mode", String, callback=moco_mode_status_callback,
                                                   queue_size=1)
    ogs1_teach_subscriber = rospy.Subscriber("/ogs_sensor/teach", String, callback=ogs_teach_callback, queue_size=1)

    ogs1_status_publisher = rospy.Publisher("/ogs1_sensor/status", Bool, queue_size=0)
    ogs2_status_publisher = rospy.Publisher("/ogs2_sensor/status", Bool, queue_size=0)
    ogs1_edges_publisher = rospy.Publisher("/ogs1_sensor/edges", Float64MultiArray, queue_size=5)
    ogs2_edges_publisher = rospy.Publisher("/ogs2_sensor/edges", Float64MultiArray, queue_size=5)
    ogs_teach_response_publisher = rospy.Publisher("/ogs_sensor/teach_status", String, queue_size=0)
    rate = rospy.Rate(60)
    # -----------------------------------------------------------------------------------------------------------

    while not rospy.is_shutdown():
#----------------------------------OGS1-------------------------------------
        if OGS1.isOpen() and (moco_mode_status == 'auto' or moco_mode_status == 'semi-auto'):
            try:
                ogs_read(OGS1, OGS1_CMD_VALID_PIXEL_READ, OGS1_CMD_INVALID_PIXEL_READ, ogs1_edges_publisher,
                         ogs1_status_publisher)
            except Exception as e:
                print(e)
                print("OGS sensor-1 read failed")
                OGS1.close()
        elif not OGS1:
            print('trying to open port')
            OGS1.open()
        else:
            OGS1.close()
            print("OGS Sensor-1 Port is not opened")
            time.sleep(2)
            OGS1.open()
            print("Opening OGS1 COM port")
#----------------------------------OGS2-------------------------------------
        if OGS2.isOpen() and (moco_mode_status == 'auto' or moco_mode_status == 'semi-auto'):
            try:
                ogs_read(OGS2, OGS2_CMD_VALID_PIXEL_READ, OGS2_CMD_INVALID_PIXEL_READ, ogs2_edges_publisher,
                         ogs2_status_publisher)
            except Exception as e:
                print(e)
                print("OGS sensor-1 read failed")
                OGS2.close()
        elif not OGS2:
            print('trying to open port')
            OGS2.open()
        else:
            OGS2.close()
            print("OGS Sensor-1 Port is not opened")
            time.sleep(2)
            OGS2.open()
            print("Opening OGS1 COM port")

        rate.sleep()
