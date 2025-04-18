#!/usr/bin/env python

import rospy
import yaml
from geometry_msgs.msg import WrenchStamped
from std_srvs.srv import Empty, EmptyResponse
from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput
from Phidget22.BridgeGain import BridgeGain


class ForceSensor:
    def __init__(self, channel_index, calib_data):
        self.channel_index = channel_index
        self.sensor = VoltageRatioInput()
        self.base_offset = calib_data['offset']
        self.zero_offset = 0.0
        self.calib_mass = calib_data.get('calibration_mass_kg', 1.0)  # default to 1.0 kg
        self.value_at_calib = calib_data['value_at_calib']
        self.scale = (self.calib_mass * 9.80665) / (self.value_at_calib - self.base_offset)
        self.publisher = None
        self.serial = calib_data.get("serial", f"CH{channel_index}")
        self.metadata = calib_data
        self._init_sensor()
        self._init_publisher()
        self._log_metadata()

    def _init_sensor(self):
        self.sensor.setChannel(self.channel_index)
        self.sensor.openWaitForAttachment(1000)
        self.sensor.setBridgeEnabled(True)
        self.sensor.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
        self.sensor.setDataInterval(self.sensor.getMinDataInterval())

    def _init_publisher(self):
        topic = f"/force_sensor/sensor_readings_{self.channel_index}"
        self.publisher = rospy.Publisher(topic, WrenchStamped, queue_size=10)

    def _log_metadata(self):
        rospy.loginfo(f"--- Sensor {self.channel_index} (Serial: {self.serial}) ---")
        for key, val in self.metadata.items():
            rospy.loginfo(f"  {key}: {val}")
        rospy.loginfo(f"  Calculated scale: {self.scale:.6f} N / raw unit")
        rospy.loginfo(f"  (Based on {self.calib_mass} kg calibration)\n")

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

        mapping = rospy.get_param("~loadcell_mapping")
        calib_path = rospy.get_param("~calibration_file")

        try:
            with open(calib_path, 'r') as f:
                all_calibs = yaml.safe_load(f)["loadcells"]
        except Exception as e:
            rospy.logfatal(f"Failed to load calibration file: {e}")
            rospy.signal_shutdown("Failed to load calibration YAML")
            return

        for i, serial in enumerate(mapping):
            if serial not in all_calibs:
                rospy.logfatal(f"[FATAL] Loadcell '{serial}' not found in calibration file!")
                rospy.signal_shutdown("Invalid loadcell mapping in launch file")
                return

            calib = all_calibs[serial]
            required_fields = ['offset', 'value_at_calib', 'calibration_mass_kg']
            missing = [field for field in required_fields if field not in calib]

            if missing:
                rospy.logfatal(
                    f"[FATAL] Calibration data for loadcell '{serial}' is missing required fields: {missing}"
                )
                rospy.signal_shutdown("Incomplete calibration data")
                return

            sensor = ForceSensor(i, calib)
            self.sensors.append(sensor)

        self.avg_publisher = rospy.Publisher("/force_sensor/average_force", WrenchStamped, queue_size=10)
        self.total_publisher = rospy.Publisher("/force_sensor/total_force", WrenchStamped, queue_size=10)
        self.service = rospy.Service("zero_all", Empty, self.handle_zero_all)
        rospy.loginfo("ForceSensorNode initialized with zeroing service.")

    def handle_zero_all(self, req):
        for sensor in self.sensors:
            sensor.zero()
        rospy.loginfo("All sensors zeroed.")
        return EmptyResponse()

    def run(self):
        while not rospy.is_shutdown():
            total_force = 0.0
            valid_count = 0

            for sensor in self.sensors:
                sensor.publish()  # Per-channel publishing

                try:
                    force = sensor.read_force()
                    total_force += force
                    valid_count += 1
                except Exception as e:
                    rospy.logwarn(f"[Sensor {sensor.channel_index}] Read error during aggregation: {e}")

            if valid_count > 0:
                avg_force = total_force / valid_count

                avg_msg = WrenchStamped()
                avg_msg.header.stamp = rospy.Time.now()
                avg_msg.header.frame_id = "force_sensor_avg"
                avg_msg.wrench.force.x = avg_force
                self.avg_publisher.publish(avg_msg)

                total_msg = WrenchStamped()
                total_msg.header.stamp = rospy.Time.now()
                total_msg.header.frame_id = "force_sensor_total"
                total_msg.wrench.force.x = total_force
                self.total_publisher.publish(total_msg)

            self.rate.sleep()
