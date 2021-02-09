#!/usr/bin/python

import unittest

import sys
import time
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink2

class fifo(object):
    def __init__(self):
        self.buf = []
    def write(self, data):
        self.buf += data
        return len(data)
    def read(self):
        return self.buf.pop(0)


def wait_conn(m):
    msg = None
    counter = 10
    #  print("Sending heartbeat",  end='')
    while not msg and counter > 0:
        m.mav.heartbeat_send(mavlink2.MAV_TYPE_QUADROTOR,
                             mavlink2.MAV_AUTOPILOT_ARDUPILOTMEGA,
                             128, 0,
                             mavlink2.MAV_STATE_ACTIVE)
        #  print(".",  end='')
        msg = m.recv_match()

        counter -= 1
        time.sleep(1)
    if counter == 0:
        print("\nTimeout!")
        return False
    print()
    return True

class CameraTests(unittest.TestCase):
    def setUp(self):
        self.camera = mavutil.mavlink_connection("/dev/tty.usbserial-00000000", baud=115200, source_system=1, source_component=1)
        self.camera.target_component = 100
        self.camera.target_system = 1

        if not wait_conn(self.camera): # Wait until we have a heartbeat
            print("Failed to connect to device")
            sys.exit(1)

    def tearDown(self) -> None:
        if self.camera is not None:
            self.camera.close()

    def test_setmode(self):

        self.camera.mav.command_long_send(
            self.camera.target_system,
            self.camera.target_component,
            mavlink2.MAV_CMD_SET_CAMERA_MODE,
            1,
            2, 0, 0, 0, 0, 0, 0)

        ack_msg = self.camera.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        self.assertIsNotNone(ack_msg, "No message received before timeout")

        ack_msg = ack_msg.to_dict()
        self.assertEqual(ack_msg['command'], mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE)
        self.assertEqual(ack_msg['result'], 0)

    def test_set_take_picture(self):
        self.camera.mav.command_long_send(
            self.camera.target_system,
            self.camera.target_component,
            mavlink2.MAV_CMD_SET_CAMERA_MODE,
            1,
            2, 0, 0, 0, 0, 0, 0)

        ack_msg = self.camera.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        self.assertIsNotNone(ack_msg, "No message received before timeout")
        ack_msg = ack_msg.to_dict()
        self.assertEqual(ack_msg['command'], mavutil.mavlink.MAV_CMD_SET_CAMERA_MODE)
        self.assertEqual(ack_msg['result'], 0)

        self.camera.mav.command_long_send(
            self.camera.target_system,
            self.camera.target_component,
            mavlink2.MAV_CMD_IMAGE_START_CAPTURE,
            1,
            0, 0, 1, 1, 0, 0, 0)

        ack_msg = self.camera.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        self.assertIsNotNone(ack_msg, "No message received before timeout")

        ack_msg = ack_msg.to_dict()
        self.assertEqual(ack_msg['command'], mavutil.mavlink.MAV_CMD_IMAGE_START_CAPTURE)
        self.assertEqual(ack_msg['result'], 0)

        capture_msg = self.camera.recv_match(type='CAMERA_IMAGE_CAPTURED', blocking=True, timeout=10)
        self.assertIsNotNone(capture_msg, "No message received before timeout")
        capture_msg = capture_msg.to_dict()
        self.assertEqual(capture_msg['capture_result'], 1)

    def test_message_camera_information(self):
        self.camera.mav.command_long_send(
            self.camera.target_system,
            self.camera.target_component,
            mavlink2.MAV_CMD_REQUEST_CAMERA_INFORMATION,
            1,
            2, 0, 0, 0, 0, 0, 0)

        ack_msg = self.camera.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
        self.assertIsNotNone(ack_msg, "No message received before timeout")
        ack_msg = ack_msg.to_dict()
        self.assertEqual(ack_msg['command'], mavutil.mavlink.MAV_CMD_REQUEST_CAMERA_INFORMATION)
        self.assertEqual(ack_msg['result'], 0)

        camera_information_msg = self.camera.recv_match(type='CAMERA_INFORMATION', blocking=True, timeout=10)
        self.assertIsNotNone(camera_information_msg, "No message received before timeout")
        camera_information_msg = camera_information_msg.to_dict()
        print(camera_information_msg)

        vendor = ""
        for x in camera_information_msg['vendor_name']:
            if x == 0:
                break
            vendor += chr(x)
        self.assertEqual(vendor, "GoPro")

if __name__ == '__main__':
    unittest.main()
