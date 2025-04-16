#!/usr/bin/env python

import rospy
import yaml
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty, EmptyResponse
from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput
from Phidget22.BridgeGain import BridgeGain

class ForceSensor:
    def __init__(self, channel_index, calibration_path):
        self.channel_index = channel_index
        self.sensor = VoltageRatioInput()
        self.base_offset = 0.0  # From YAML
        self.zero_offset = 0.0  # Dynamic zero
        self.scale = 1.0
        self.publisher = None

        self._load_calibration(calibration_path)
        self._init_sensor()
        self._init_publisher()

    def _load_calibration(self, path):
        with open(path, 'r') as f:
            calib = yaml.safe_load(f)
        self.base_offset = calib['offset']
        raw_1kg = calib['value_at_1kg']
        self.scale = 9.80665 / (raw_1kg - self.base_offset)  # Converts to N

    def _init_sensor(self):
        self.sensor.setChannel(self.channel_index)
        self.sensor.openWaitForAttachment(1000)
        self.sensor.setBridgeEnabled(True)
        self.sensor.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
        self.sensor.setDataInterval(self.sensor.getMinDataInterval())

    def _init_publisher(self):
        topic = f"/force_sensor/sensor_readings_{self.channel_index}"
        self.publisher = rospy.Publisher(topic, WrenchStamped, queue_size=10)

    def read_force(self):
        raw = self.sensor.getVoltageRatio()
        return (raw - self.base_offset - self.zero_offset) * self.scale

    def zero(self):
        try:
            current = self.sensor.getVoltageRatio()
            self.zero_offset = current - self.base_offset
            rospy.loginfo(f"[Sensor {self.channel_index}] Zeroed with offset: {self.zero_offset:.6f}")
        except Exception as e:
            rospy.logwarn(f"[Sensor {self.channel_index}] Zeroing failed: {e}")

    def publish(self):
        try:
            force = self.read_force()
            msg = WrenchStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = f"force_sensor_{self.channel_index}"
            msg.wrench.force.x = force
            self.publisher.publish(msg)
        except Exception as e:
            rospy.logwarn(f"[Sensor {self.channel_index}] Read error: {e}")


class ForceSensorNode:
    def __init__(self):
        rospy.init_node("force_sensor_node")
        self.sensors = []
        self.rate = rospy.Rate(125)

        for i in range(4):
            calib_param = f"~calibration_file_{i}"
            calib_path = rospy.get_param(calib_param)
            sensor = ForceSensor(i, calib_path)
            self.sensors.append(sensor)

        self.service = rospy.Service("zero_all", Empty, self.handle_zero_all)
        rospy.loginfo("ForceSensorNode initialized with zeroing service.")

    def handle_zero_all(self, req):
        for sensor in self.sensors:
            sensor.zero()
        rospy.loginfo("All sensors zeroed.")
        return EmptyResponse()

    def run(self):
        while not rospy.is_shutdown():
            for sensor in self.sensors:
                sensor.publish()
            self.rate.sleep()

