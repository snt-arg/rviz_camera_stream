#!/usr/bin/env python
import rospy, argparse
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class IncrementalVideoRecorder:
    def __init__(self, args):
        self.camera_name = args.camera_name
        self.log_path = args.log_path
        rospy.loginfo(f"flag {args.log_path}")
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber(f"/rviz1/{args.camera_name}/image", Image, self.callback)
        self.segment_length = 1000
        self.frame_count = 0
        self.segment_count = 0
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.frame_width = 2560  # Adjust based on your camera's resolution
        self.frame_height = 1440  # Adjust based on your camera's resolution
        self.new_segment()

    def new_segment(self):
        # Increment segment count and create a new video writer object
        self.segment_count += 1
        video_filename = f'{self.log_path}/{self.camera_name}/output_segment_{self.segment_count}.avi'
        self.out = cv2.VideoWriter(video_filename, self.fourcc, 20.0, (self.frame_width, self.frame_height))
        self.frame_count = 0

    def callback(self, data):
        if self.frame_count >= self.segment_length:
            # Save the current segment and start a new one
            self.save_video()
            self.new_segment()

        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            # Write the image to a video file
            self.out.write(cv2_img)
            self.frame_count += 1

    def save_video(self):
        # Release the video writer object
        self.out.release()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='')
    parser.add_argument('-log_path', type=str,
                        help='an integer for the accumulator')
    parser.add_argument('-camera_name', type=str,
                        help='an integer for the accumulator')
    parser.add_argument('__name')
    parser.add_argument('__log')

    args = parser.parse_args()
    rospy.init_node(f'incremental_video_recorder_{args.camera_name}', anonymous=True)
    vr = IncrementalVideoRecorder(args)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        vr.save_video()


