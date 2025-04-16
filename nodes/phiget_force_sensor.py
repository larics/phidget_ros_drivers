#!/usr/bin/env python
from phidget_ros_drivers import ForceSensorNode

if __name__ == "__main__":
    try:
        ForceSensorNode().run()
    except rospy.ROSInterruptException:
        pass


