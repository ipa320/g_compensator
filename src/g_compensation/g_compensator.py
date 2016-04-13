#!/usr/bin/env python
import rospy
import PyKDL as kdl
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
    if not tf2_buffer.can_transform(com_frame, gravity_frame, rospy.Time.now(),
                                    rospy.Duration(1.0)):
        rospy.logwarn('Could not get an initial transform from %s to %s.',
                      com_frame, gravity_frame)

    def wrench_cb(msg):
        time = rospy.Time(0)  # alternative: msg.header.stamp
        force_frame = msg.header.frame_id
        # get transforms: gravity -> com -> sensor
        tf_gravity = get_frame(com_frame, gravity_frame, time)
        tf_com = get_frame(force_frame, com_frame, time)
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
