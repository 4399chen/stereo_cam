import cv2
import sys
import vpi
import numpy as np
from PIL import Image as PilImage
# from argparse import ArgumentParser

import rospy
from sensor_msgs.msg import Image as MsgImage
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import threading
from concurrent.futures import ThreadPoolExecutor


class StereoImageProcessor:
    def __init__(self):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/camera/combined_image', MsgImage, queue_size=1)
        
        # 创建线程池
        self.executor = ThreadPoolExecutor(max_workers=8)  # 可以根据需要调整线程数量

        # 创建订阅者
        left_image_sub = message_filters.Subscriber('/camera/left/image_rect', MsgImage)
        right_image_sub = message_filters.Subscriber('/camera/right/image_rect', MsgImage)

        # 使用 message_filters 进行同步
        ts = message_filters.TimeSynchronizer([left_image_sub, right_image_sub], 10)
        ts.registerCallback(self.callback) 
    
    def callback(self, left_img_msg, right_img_msg):
        # 将处理函数提交到线程池
        self.executor.submit(self.process_images, left_img_msg, right_img_msg)

    def process_images(self, left_img_msg, right_img_msg):
        left_img = self.bridge.imgmsg_to_cv2(left_img_msg, "mono16")
        right_img = self.bridge.imgmsg_to_cv2(right_img_msg, "mono16")

        left_img_resized = cv2.resize(left_img, (480, 270))
        right_img_resized = cv2.resize(right_img, (480, 270))

        backend = vpi.Backend.PVA
        conftype = None
        conftype = vpi.ConfidenceType.ABSOLUTE
    
        # Streams for left and right independent pre-processing
        streamLeft = vpi.Stream()
        streamRight = vpi.Stream()
    
        #  
        with vpi.Backend.CUDA:
            with streamLeft:
                left = vpi.asimage(left_img_resized)
            with streamRight:
                right = vpi.asimage(right_img_resized)

        # Use stream left to consolidate actual stereo processing
        streamStereo = streamLeft
    
        # Estimate stereo disparity.
        with streamStereo, backend:
            disparityS16 = vpi.stereodisp(left, right, downscale=1,
                                        window=5, maxdisp=64, confthreshold=31000,
                                        quality=6, conftype=conftype, mindisp=0,
                                        p1=3, p2=48, p2alpha=0, uniqueness=-1,
                                        includediagonals=True, numpasses=3)
    
        # Postprocess results
        with streamStereo, vpi.Backend.CUDA:
            # Some backends outputs disparities in block-linear format, we must convert them to
            # pitch-linear for consistency with other backends.
            if disparityS16.format == vpi.Format.S16_BL:
                disparityS16 = disparityS16.convert(vpi.Format.S16, backend=vpi.Backend.VIC)
    
            # Scale disparity and confidence map so that values like between 0 and 255.
            disparityU8 = disparityS16.convert(vpi.Format.U8, scale=255.0/(32*64)).cpu()
            disparityU8 = cv2.medianBlur(disparityU8, ksize=5)
            disparityColor = cv2.applyColorMap(disparityU8, cv2.COLORMAP_JET)
        
        combined_img_msg = self.bridge.cv2_to_imgmsg(disparityColor, "bgr8")
        combined_img_msg.header.stamp = rospy.Time.now()
        self.pub.publish(combined_img_msg)
    
    def close(self):
        self.executor.shutdown(wait=True)

def main():
    rospy.init_node('stereo_image_combiner', anonymous=True)
    processor = StereoImageProcessor()
    try:
        rospy.spin()
    finally:
        processor.close()

if __name__ == '__main__':
    main()

