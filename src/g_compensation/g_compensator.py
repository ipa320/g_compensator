#!/usr/bin/env python
import collections
import threading

import rospy
import PyKDL as kdl
import tf2_py as tf2
import tf2_kdl
import tf2_ros
import geometry_msgs.msg as geometry_msgs
from std_srvs.srv import Empty, EmptyResponse
from functools import reduce

tf2_buffer = tf2_ros.Buffer()


def mean_wrench(wrenches):
    return reduce(lambda x, y: x + y, wrenches) / len(wrenches)


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
        rospy.logerr_once('Waiting for transform "%s" -> "%s".', parent, frame)


def get_frame(parent, frame, time):
    return tf2_kdl.transform_to_kdl(
        tf2_buffer.lookup_transform(parent, frame, time))


if __name__ == '__main__':
    rospy.init_node('g_compensator', anonymous=True)

    pub = rospy.Publisher('wrench_compensated', geometry_msgs.WrenchStamped,
                          queue_size=1)

    # start filling the tf buffer
    tf2_listener = tf2_ros.TransformListener(tf2_buffer)

    # get parameters
    mass = rospy.get_param('~mass')
    gravity_frame = rospy.get_param('~gravity_frame', 'world')
    com_frame = rospy.get_param('~com_frame')  # com = center of mass
    negate_wrench = -1 if rospy.get_param('~negate_wrench', False) else 1

    gravity = kdl.Wrench(kdl.Vector(0, 0, -9.81*mass), kdl.Vector(0, 0, 0))
    tare_offset = kdl.Wrench(kdl.Vector(0, 0, 0), kdl.Vector(0, 0, 0))
    run_tare = False

    # wait for initial transform
    init_transform(com_frame, gravity_frame)

    # tare
    wrench_buffer = collections.deque(maxlen=50)
    run_tare = threading.Event()

    def wrench_cb(msg):
        global tare_offset

        time = rospy.Time(0)  # alternative: msg.header.stamp
        force_frame = msg.header.frame_id

        # get transforms: gravity -> com -> sensor
        try:
            tf_gravity = get_frame(com_frame, gravity_frame, time)
            tf_com = get_frame(force_frame, com_frame, time)
        except tf2.TransformException as e:
            rospy.logerr(e)
            init_transform(com_frame, gravity_frame)
            init_transform(force_frame, com_frame)
            return

        # compute gravity at com: change coordinate system (rotate)
        gravity_at_com = tf_gravity.M * gravity

        # compute gravity at sensor: screw theory transform
        # this covers coordinate change and translation-lever
        gravity_at_sensor = tf_com * gravity_at_com

        # compensate
        compensated = negate_wrench * wrench_msg_to_kdl(msg) - gravity_at_sensor
        wrench_buffer.append(compensated)

        # Tare
        if run_tare.is_set():
            tare_offset = mean_wrench(wrench_buffer)
            run_tare.clear()
            rospy.loginfo("Tared sensor")

        compensated = compensated - tare_offset

        # publish
        compensated_msg = geometry_msgs.WrenchStamped(
            header=msg.header, wrench=wrench_kdl_to_msg(compensated))
        pub.publish(compensated_msg)

    def tare_cb(req):
        run_tare.set()
        return EmptyResponse()

    # pubs'n'subs
    rospy.Subscriber('wrench', geometry_msgs.WrenchStamped, wrench_cb,
                     queue_size=1)
    rospy.Service('tare', Empty, tare_cb)

    # run
    rospy.loginfo("Running gravity compensator")
    rospy.spin()

    rospy.loginfo("Shutting down gravity compensator")
