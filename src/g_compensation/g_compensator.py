#!/usr/bin/env python
import threading

import rospy
import PyKDL as kdl
import tf2_py as tf2
import tf2_kdl
import tf2_ros
import geometry_msgs.msg as geometry_msgs

tf2_buffer = tf2_ros.Buffer()


def wrench_msg_to_kdl(msg):
    f = msg.wrench.force
    t = msg.wrench.torque
    return kdl.Wrench(kdl.Vector(f.x, f.y, f.z), kdl.Vector(t.x, t.y, t.z))


def wrench_kdl_to_msg(w):
    return geometry_msgs.Wrench(
        force=geometry_msgs.Vector3(*w.force),
        torque=geometry_msgs.Vector3(*w.torque))


def init_transform(parent, frame):
    while (not tf2_buffer.can_transform(
               parent, frame, rospy.Time.now(), rospy.Duration(4.0)) and
           not rospy.is_shutdown()):
        rospy.logerr('Waiting for transform "%s" -> "%s".', parent, frame)


def get_frame(parent, frame, time):
    return tf2_kdl.transform_to_kdl(
        tf2_buffer.lookup_transform(parent, frame, time))


if __name__ == '__main__':
    rospy.init_node('g_compensator', anonymous=True)
    pub = rospy.Publisher('/wrench_compensated', geometry_msgs.WrenchStamped,
                          queue_size=1)
    # start filling the tf buffer
    tf2_listener = tf2_ros.TransformListener(tf2_buffer)
    # get parameters
    mass = rospy.get_param('~mass')
    gravity = kdl.Wrench(kdl.Vector(0, 0, -9.81*mass), kdl.Vector(0, 0, 0))
    gravity_frame = rospy.get_param('~gravity_frame', 'world')
    # com = center of mass
    com_frame = rospy.get_param('~com_frame')
    # wait for initial transform
    init_transform(com_frame, gravity_frame)
    tf_available = threading.Event()
    tf_available.set()

    def wrench_cb(msg):
        if not tf_available.is_set():
            rospy.logdebug('Unavailable transforms: Skipping message.')
            return
        time = rospy.Time(0)  # alternative: msg.header.stamp
        force_frame = msg.header.frame_id
        # get transforms: gravity -> com -> sensor
        try:
            tf_gravity = get_frame(com_frame, gravity_frame, time)
            tf_com = get_frame(force_frame, com_frame, time)
        except tf2.TransformException as e:
            tf_available.clear()
            rospy.logerr(e)
            init_transform(com_frame, gravity_frame)
            init_transform(force_frame, com_frame)
            tf_available.set()
            return
        # compute gravity at com: change coordinate system (rotate)
        gravity_at_com = tf_gravity.M * gravity
        # compute gravity at sensor: screw theory transform
        # this covers coordinate change and translation-lever
        gravity_at_sensor = tf_com * gravity_at_com
        # compensate
        compensated = wrench_msg_to_kdl(msg) - gravity_at_sensor
        # publish
        compensated_msg = geometry_msgs.WrenchStamped(
            header=msg.header, wrench=wrench_kdl_to_msg(compensated))
        pub.publish(compensated_msg)

    rospy.Subscriber('/wrench', geometry_msgs.WrenchStamped, wrench_cb)
    rospy.spin()
