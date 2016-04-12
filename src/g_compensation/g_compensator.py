#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy

import PyKDL as kdl
import tf2_py
import tf2_kdl
import tf2_ros
from geometry_msgs.msg import Vector3, Wrench, WrenchStamped

tf2_buffer = tf2_ros.Buffer()


def wrench_msg_to_kdl(msg):
    f = msg.wrench.force
    t = msg.wrench.torque
    return kdl.Wrench(kdl.Vector(f.x, f.y, f.z), kdl.Vector(t.x, t.y, t.z))


def wrench_kdl_to_msg(w):
    return Wrench(force=Vector3(*w.force), torque=Vector3(*w.torque))


if __name__ == '__main__':
    rospy.init_node('force_listener', anonymous=True)

    pub = rospy.Publisher('/wrench_compensated', WrenchStamped, queue_size=1)

    tf2_listener = tf2_ros.TransformListener(tf2_buffer)

    mass = rospy.get_param('~mass')
    gravity = kdl.Wrench(kdl.Vector(0, 0, -9.81*mass), kdl.Vector(0, 0, 0))
    gravity_frame = 'world'
    com_frame = rospy.get_param('~com_frame')

    def wrench_cb(msg):
        try:
            tf_gravity = tf2_kdl.transform_to_kdl(
                tf2_buffer.lookup_transform(com_frame, gravity_frame, rospy.Time(0)))
            tf_com = tf2_kdl.transform_to_kdl(
                tf2_buffer.lookup_transform(msg.header.frame_id, com_frame, rospy.Time(0)))
        except (
                tf2_py.ConnectivityException,
                tf2_py.ExtrapolationException,
                tf2_py.LookupException) as e:
            rospy.logwarn('Waiting for transform.')
            if tf2_buffer.can_transform(msg.header.frame_id, gravity_frame, rospy.Time(0), rospy.Duration(1.0)):
                rospy.logwarn('Got transform.')
            else:
                rospy.logerr(e)
            return
        gravity_at_com = tf_gravity.M * gravity
        gravity_at_sensor = tf_com * gravity_at_com
        compensated = wrench_msg_to_kdl(msg) - gravity_at_sensor
        compensated_msg = WrenchStamped(header=msg.header, wrench=wrench_kdl_to_msg(compensated))
        pub.publish(compensated_msg)

    rospy.Subscriber("/ipa325_ftcl_can_node/wrench", WrenchStamped, wrench_cb)
    rospy.spin()