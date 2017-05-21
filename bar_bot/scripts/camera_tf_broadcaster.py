#!/usr/bin/env python  
import roslib
import numpy
roslib.load_manifest('camera_tf')

import rospy
import tf
from tf import transformations as t

#-0.0368 0.0524
quaternion_n = t.quaternion_from_euler(0.05240000000, 0.29640000000, 0.00600000000)
if __name__ == '__main__':
    rospy.init_node('bar_bot_tf_broadcaster')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.26883453, 0.07474525, 0.51304644),
                         quaternion_n,
                         rospy.Time.now(),
                         "camera_link",
                         "base")
        rate.sleep()


