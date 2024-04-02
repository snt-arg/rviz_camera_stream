#!/usr/bin/env python
import rospy, json, time
import tf
import tf2_ros
import geometry_msgs.msg
import math
import numpy as np
import tf.transformations
import argparse



def quaternion_multiply(q1, q2):
    """
    Multiplies two quaternions.
    q1, q2: Arrays representing quaternions (x, y, z, w)
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 + y1*w2 + z1*x2 - x1*z2
    z = w1*z2 + z1*w2 + x1*y2 - y1*x2

    return np.array([x, y, z, w])

class CamerasFrameManager():
    def __init__(self, args):
        rospy.init_node('virtual_camera_manager')

        rospy.loginfo(args.settings_path)
        rospy.loginfo(args.dataset_type)
        rospy.loginfo(args.dataset_tag)

        self.rate = rospy.Rate(10.0)

        with open(args.settings_path + "/dataset_settings.json") as f:
            self.camera_settings = json.load(f)[args.dataset_type][args.dataset_tag]["cameras"]

        if "top" in self.camera_settings.keys():
            self.publish_top_keyframe()
        if "obli" in self.camera_settings.keys():
            self.publish_obli_keyframe()
        if "rotary" in self.camera_settings.keys():
            self.publish_rotary_keyframe()

    def publish_top_keyframe(self):

        brs1 = tf2_ros.StaticTransformBroadcaster()
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "camera_top"
        transform.transform.translation.x = self.camera_settings["center"][0]
        transform.transform.translation.y = self.camera_settings["center"][1]
        transform.transform.translation.z = self.camera_settings["top"]["height"]

        quaternion1 = tf.transformations.quaternion_from_euler(0, 0, math.radians(self.camera_settings["top"]["rotation_angle"]))
        quaternion2 = tf.transformations.quaternion_from_euler(0, math.radians(180), 0)
        quaternion4 = quaternion_multiply(quaternion1, quaternion2)
        quaternion3 = tf.transformations.quaternion_from_euler(0, 0, math.radians(90))
        quaternion = quaternion_multiply(quaternion4, quaternion3)
        
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]

        brs1.sendTransform(transform)
        time.sleep(1)

    def publish_obli_keyframe(self):
        
        brs = tf2_ros.StaticTransformBroadcaster()
        transform = geometry_msgs.msg.TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "map"
        transform.child_frame_id = "camera_obli"

        radius = self.camera_settings["obli"]["radius"]
        height = self.camera_settings["obli"]["height"]
        center_x = self.camera_settings["center"][0]
        center_y = self.camera_settings["center"][1]
        angle_to_center = math.radians(self.camera_settings["obli"]["angle_to_center"])
        x = center_x + radius * math.cos(angle_to_center)
        y = center_y + radius * math.sin(angle_to_center)
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = height

        inclination_angle = math.radians(self.camera_settings["obli"]["inclination_angle"])

        quaternion1 = tf.transformations.quaternion_from_euler(0, inclination_angle, angle_to_center)
        quaternion2 = tf.transformations.quaternion_from_euler(0, math.radians(180), 0)
        quaternion4 = quaternion_multiply(quaternion1, quaternion2)
        quaternion3 = tf.transformations.quaternion_from_euler(0, 0, math.radians(90))
        quaternion = quaternion_multiply(quaternion4, quaternion3)
        
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        
        brs.sendTransform(transform)
        self.rate.sleep()

    def publish_rotary_keyframe(self):
        
        camera_settings = self.camera_settings["rotary"]
        br = tf2_ros.TransformBroadcaster()

        # Speed parameter: Adjust to change the rotation speed around the circle
        cycle_time = camera_settings["cycle_time"]
        speed = 2 * math.pi / cycle_time # Circles per second (rad/s)
        inclination_angle = camera_settings["inclination_angle"]  # degrees of inclination towards the center
        init_t = None
        previous_angle = 1
        laps = 0

        while not rospy.is_shutdown():
            # Current time adjusted by speed for circular motion
            t_now = rospy.Time.now().to_sec()
            if not init_t:
                init_t = t_now
            t_rel_speed = (t_now - init_t) * speed

            # Circular path parameters
            radius = camera_settings["radius"]
            height = camera_settings["height"]
            center_x = self.camera_settings["center"][0]
            center_y = self.camera_settings["center"][1]

            x = center_x + radius * math.cos(t_rel_speed)
            y = center_y + radius * math.sin(t_rel_speed)

            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "map"
            transform.child_frame_id = "camera_rotary"
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = height

            # Applying inclination
            inclination = math.radians(inclination_angle)

            angle_to_center = math.atan2(math.sin(t_rel_speed), math.cos(t_rel_speed))
            if (previous_angle < 0) and angle_to_center > 0:
                laps += 1
            previous_angle = angle_to_center

            # print(f"VC spinner: angle_to_center {angle_to_center} laps {laps}")
            # Combine the rotation to face the center with the inclination
            quaternion1 = tf.transformations.quaternion_from_euler(0, inclination, angle_to_center)
            quaternion2 = tf.transformations.quaternion_from_euler(0, math.radians(180), 0)
            quaternion4 = quaternion_multiply(quaternion1, quaternion2)
            quaternion3 = tf.transformations.quaternion_from_euler(0, 0, math.radians(90))
            quaternion = quaternion_multiply(quaternion4, quaternion3)
            
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]

            br.sendTransform(transform)
            # if laps > 0:
            #     return
            self.rate.sleep()

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='')
    parser.add_argument('-settings_path', type=str,
                        help='an integer for the accumulator')
    parser.add_argument('-dataset_type', type=str,
                        help='an integer for the accumulator')
    parser.add_argument('-dataset_tag', type=str,
                        help='an integer for the accumulator')
    parser.add_argument('__name')
    parser.add_argument('__log')

    args = parser.parse_args()
    cfm = CamerasFrameManager(args)
    # try:
    #     publish_rotary_keyframe()
    # except rospy.ROSInterruptException:
    #     pass


    # {
    #     "set_tag" : "OursFull",
    #     "generation_method" : "reasoning",
    #     "reasoning_generated_entities" : ["wall", "room"],
    #     "cameras" : ["rotary"]
    # },

    # {
    #     "set_tag" : "voxblox",
    #     "generation_method" : "voxblox",
    #     "reasoning_generated_entities" : ["room"],
    #     "cameras" : ["rotary"]
    # },