#!/usr/bin/env python

## @file phidget_calibration.py
#  @brief Calibrates Phidget load cell sensors by determining sensitivity and offset per selected channel.

import rospy
import time
from enum import Enum
from Phidget22.Devices.VoltageRatioInput import VoltageRatioInput
from Phidget22.BridgeGain import BridgeGain
from Phidget22.PhidgetException import PhidgetException

## @enum SensorState
#  @brief Enum representing calibration state of each sensor channel.
class SensorState(Enum):
    WAITING = 0        # < Waiting for weight to be placed
    STABILIZING = 1    # < Weight placed, waiting for reading to stabilize
    CALIBRATED = 2     # < Sensor calibrated

## @class CalibrationNode
#  @brief Handles the initialization and calibration process for multiple Phidget sensor channels.
class CalibrationNode:
    ## @brief Constructor that initializes sensors, sets parameters, and prepares internal state.
    def __init__(self):
        rospy.init_node('calibration_node')

        ## Calibration sensitivity tolerance
        self.tolerance = rospy.get_param("tolerance", 0.001)
        ## Weight in kg (or desired units) used for calibration
        self.known_weight = rospy.get_param("known_weight", 1.0)
        raw_channels = rospy.get_param("channels", [0, 1, 2, 3])
        ## List of sensor channels to calibrate
        self.channels = [int(c) for c in raw_channels]

        rospy.loginfo(f"Selected channels for calibration: {self.channels}")

        # Sensor setup and internal buffers
        self.sensors = {}        # VoltageRatioInput objects per channel
        self.offsets = {}        # Tare values per channel
        self.sensitivities = {}  # Sensitivity values per channel
        self.states = {}         # SensorState for each channel
        self.baselines = {}      # Baseline reading after detecting weight
        self.buffers = {}        # Buffer to average stabilized values
        self.start_times = {}    # Time when stabilization began
        self.last_log_times = {} # Last time a live reading was logged

        for i in self.channels:
            try:
                sensor = VoltageRatioInput()
                sensor.setChannel(i)
                sensor.openWaitForAttachment(1000)
                sensor.setBridgeEnabled(True)
                sensor.setBridgeGain(BridgeGain.BRIDGE_GAIN_128)
                sensor.setDataInterval(sensor.getMinDataInterval())
                self.sensors[i] = sensor

                # Initialize state tracking
                self.offsets[i] = 0.0
                self.sensitivities[i] = None
                self.states[i] = SensorState.WAITING
                self.baselines[i] = None
                self.buffers[i] = []
                self.start_times[i] = None
                self.last_log_times[i] = 0.0

            except PhidgetException as e:
                rospy.logwarn(f"Channel {i} could not be initialized: {e}")

        rospy.loginfo("Calibration node initialized.")

    ## @brief Reads current voltage ratio from all active channels.
    #  @return Dictionary of {channel: voltage_ratio}
    def read_all(self):
        return {i: self.sensors[i].getVoltageRatio() for i in self.channels if i in self.sensors}

    ## @brief Tares all selected sensors by averaging stable values to determine zero-offset.
    def tare_sensors(self):
        rospy.loginfo("Step 1: Taring sensors (averaging over 5 seconds)...")

        buffers = {i: [] for i in self.channels}
        start_time = time.time()

        while time.time() - start_time < 5.0 and not rospy.is_shutdown():
            readings = self.read_all()
            for i in self.channels:
                buffers[i].append(readings[i])
            rospy.loginfo_throttle(3, f"Tare readings: {['{:.11f}'.format(readings[i]) for i in self.channels]}")
            time.sleep(0.05)  # 20 Hz

        self.offsets = {
            i: sum(buffers[i]) / len(buffers[i]) if buffers[i] else 0.0
            for i in self.channels
        }

        rospy.loginfo("Tare complete.")
        rospy.loginfo(f"Offsets: {['{:.11f}'.format(self.offsets[i]) for i in self.channels]}")

    ## @brief Performs the main calibration logic for all selected channels.
    def calibrate_weight(self):
        rospy.loginfo("\nStep 2: Place weights one at a time. Detecting deviation from tare...")

        last_status_print = time.time()

        while not all(self.sensitivities[i] is not None for i in self.channels) and not rospy.is_shutdown():
            now = time.time()
            readings = self.read_all()

            for i in self.channels:
                if i not in readings or self.sensitivities[i] is not None:
                    continue

                reading = readings[i]
                offset = self.offsets[i]
                state = self.states[i]

                if state == SensorState.WAITING:
                    if abs(reading - offset) > self.tolerance:
                        self.states[i] = SensorState.STABILIZING
                        self.baselines[i] = reading
                        self.buffers[i] = []
                        self.start_times[i] = None
                        rospy.loginfo(f"[Sensor {i}] Weight detected! Entering stabilization...")

                elif state == SensorState.STABILIZING:
                    baseline = self.baselines[i]
                    in_tol = abs(reading - baseline) <= self.tolerance

                    if now - self.last_log_times[i] > 0.5:
                        state_msg = "IN tolerance" if in_tol else "OUT of tolerance"
                        rospy.loginfo(f"[Sensor {i}] Reading: {reading:.11f} | Baseline: {baseline:.11f} | {state_msg}")
                        self.last_log_times[i] = now

                    if in_tol:
                        if self.start_times[i] is None:
                            self.start_times[i] = now
                        self.buffers[i].append(reading)

                        if now - self.start_times[i] >= 3.0:
                            avg = sum(self.buffers[i]) / len(self.buffers[i])
                            sensitivity = (avg - offset) / self.known_weight
                            self.sensitivities[i] = sensitivity
                            self.states[i] = SensorState.CALIBRATED
                            rospy.loginfo(f"[Sensor {i}] Calibrated! Sensitivity: {sensitivity:.11f}")
                    else:
                        self.baselines[i] = reading
                        self.buffers[i] = []
                        self.start_times[i] = None

            if now - last_status_print > 15.0:
                rospy.loginfo("=== Calibration Status ===")
                for i in self.channels:
                    s = self.sensitivities[i]
                    state = "DONE" if s is not None else self.states[i].name
                    rospy.loginfo(f"Sensor {i}: {state}")
                last_status_print = now

            time.sleep(0.05)

        rospy.loginfo("\nAll selected sensors calibrated!")
        rospy.loginfo("Final offsets and sensitivities:")
        for i in self.channels:
            rospy.loginfo(f"  Sensor {i}: Offset = {self.offsets[i]:.11f}, Sensitivity = {self.sensitivities[i]:.11f}")

    ## @brief Runs the full calibration procedure.
    def run(self):
        self.tare_sensors()
        self.calibrate_weight()

## @brief Entry point: creates and runs the calibration node.
if __name__ == "__main__":
    try:
        CalibrationNode().run()
    except rospy.ROSInterruptException:
        pass
