#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
 
termination = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03)
feature_params = dict(maxCorners=200, qualityLevel=0.01, minDistance=7, blockSize=7)
lk_params = dict(winSize=(15,15), maxLevel=2, criteria=termination)

class Optical_flow():
    def __init__(self):
        self.selecting_sub_image = "raw" # you can choose image type "compressed", "raw"
 
        if self.selecting_sub_image == "compressed":
            self._sub = rospy.Subscriber('/usb_cam/image_raw/compressed', CompressedImage, self.callback, queue_size=1)
        else:
            self._sub = rospy.Subscriber('/usb_cam/image_raw', Image, self.callback, queue_size=1)
 
        self.bridge = CvBridge()
        self.track_len = 10
        self.detect_interval = 5
        self.tracks = []
        self.frame_idx = 0
        self.blackscreen = False

    def callback(self, image_msg):
 
        if self.selecting_sub_image == "compressed":
            #converting compressed image to opencv image
            np_arr = np.fromstring(image_msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        elif self.selecting_sub_image == "raw":
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
 
        frame_gray = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
       
        vis = cv_image.copy()


        if len(self.tracks) > 0:
            img0, img1 = self.prev_gray, frame_gray
            p0 = np.float32([tr[-1] for tr in self.tracks]).reshape(-1,1,2)
            p1, st, err = cv2.calcOpticalFlowPyrLK(img0, img1, p0, None, **lk_params)
            p0r, st, err = cv2.calcOpticalFlowPyrLK(img1, img0, p1, None, **lk_params)
            d = abs(p0-p0r).reshape(-1,2).max(-1)
            good = d<1
            new_tracks = []
            for tr, (x,y), good_flag in zip(self.tracks, p1.reshape(-1,2), good):
                if not good_flag:
                    continue
                tr.append((x,y))
                if len(tr) > self.track_len:
                    del tr[0]

                new_tracks.append(tr)
                cv2.circle(vis, (x,y), 2, (0,255,0), -1)

            self.tracks = new_tracks
            cv2.polylines(vis, [np.int32(tr) for tr in self.tracks], False, (0,255,0))

        if self.frame_idx % self.detect_interval == 0:
            mask = np.zeros_like(frame_gray)
            mask[:] = 255
            for x,y in [np.int32(tr[-1]) for tr in self.tracks]:
                cv2.circle(mask, (x,y), 5, 0, -1)
            p = cv2.goodFeaturesToTrack(frame_gray, mask=mask, **feature_params)

            if p is not None:
                for x,y in np.float32(p).reshape(-1,2):
                    self.tracks.append([(x,y)])

        self.frame_idx += 1
        self.prev_gray = frame_gray

        cv2.imshow('optical_flow', vis)
        cv2.waitKey(1)

    def main(self):
        rospy.spin()
 
if __name__ == '__main__':
    rospy.init_node('optical_flow')
    node = Optical_flow()
    node.main()
