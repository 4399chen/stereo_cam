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

class StereoImageProcessor:
    def __init__(self):
        self.backend         = 'cuda'                        #     choices=['cpu','cuda','pva','ofa','ofa-pva-vic','pva-nvenc-vic'],help='Backend to be used for process
        self.left            = '/camera/left/image_rect'    #     type=ros topic, help='Input left ROS topic'
        self.right           = '/camera/right/image_rect'   #     type=ros topic, help='Input right ROS topic'
        self.width           = -1                           #     default=-1, type=int, help='Input width for raw input files'
        self.height          = -1                           #     default=-1, type=int, help='Input height for raw input files'
        self.downscale       = 1                            #     default=1, type=int, help='Output downscale factor'
        self.window_size     = 5                            #     default=5, type=int, help='Median filter window size'
        self.skip_confidence = False                        #     default=False, action='store_true', help='Do not calculate confidence'
        self.conf_threshold  = 32767                        #     default=32767, type=int, help='Confidence threshold'
        self.conf_type       = 'absolute'                   #     default='absolute', choices=['absolute', 'relative'],help='Computation type to produce the confide
        self.p1              = 3                            #     default=3, type=int, help='Penalty P1 on small disparities'
        self.p2              = 48                           #     default=48, type=int, help='Penalty P2 on large disparities'
        self.p2_alpha        = 0                            #     default=0, type=int, help='Alpha for adaptive P2 Penalty'
        self.uniqueness      = -1                           #     default=-1, type=float, help='Uniqueness ratio'
        self.skip_diagonal   = False                        #     default=False, action='store_true', help='Do not use diagonal paths'
        self.num_passes      = 3                            #     default=3, type=int, help='Number of passes'
        self.min_disparity   = 0                            #     default=0, type=int, help='Minimum disparity'
        self.max_disparity   = 256                          #     default=256, type=int, help='Maximum disparity'
        self.output_mode     = 0                            #     default=0, type=int, help='0: color; 1: grayscale'
        self.verbose         = False                        #     default=False, help='Verbose mode'
        
        self.bridge = CvBridge()
        self.pub_disp = rospy.Publisher('/camera/disp_image', MsgImage, queue_size=1)
        self.pub_conf = rospy.Publisher('/camera/conf_image', MsgImage, queue_size=1)

        # 创建订阅者
        left_image_sub  = message_filters.Subscriber(self.left,  MsgImage)
        right_image_sub = message_filters.Subscriber(self.right, MsgImage)

        # 使用 message_filters 进行同步
        ts = message_filters.TimeSynchronizer([left_image_sub, right_image_sub], 10)
        ts.registerCallback(self.callback) 
    
    # @profile
    def callback(self, left_img_msg, right_img_msg):

        scale = 1 # pixel value scaling factor when loading input
    
        if self.backend == 'cpu':
            backend = vpi.Backend.CPU
        elif self.backend == 'cuda':
            backend = vpi.Backend.CUDA
        elif self.backend == 'pva':
            backend = vpi.Backend.PVA
        elif self.backend == 'ofa':
            backend = vpi.Backend.OFA
        elif self.backend == 'ofa-pva-vic':
            backend = vpi.Backend.OFA|vpi.Backend.PVA|vpi.Backend.VIC
        elif self.backend == 'pva-nvenc-vic':
            backend = vpi.Backend.PVA|vpi.Backend.NVENC|vpi.Backend.VIC
            # For PVA+NVENC+VIC mode, 16bpp input must be MSB-aligned, which
            # is equivalent to say that it is Q8.8 (fixed-point, 8 decimals).
            scale = 256
        else:
            raise ValueError(f'E Invalid backend: {self.backend}')
    
        conftype = None
        if self.conf_type == 'absolute':
            conftype = vpi.ConfidenceType.ABSOLUTE
        elif self.conf_type == 'relative':
            conftype = vpi.ConfidenceType.RELATIVE
        else:
            raise ValueError(f'E Invalid confidence type: {self.conf_type}')
    
        minDisparity = self.min_disparity
        maxDisparity = self.max_disparity
        includeDiagonals = not self.skip_diagonal
        numPasses = self.num_passes
        calcConf = not self.skip_confidence
        downscale = self.downscale
        windowSize = self.window_size
        quality = 6
    
        if self.verbose:
            print(f'I Backend: {backend}\nI Left image: {self.left}\nI Right image: {self.right}\n'
                f'I Disparities (min, max): {(minDisparity, maxDisparity)}\n'
                f'I Input scale factor: {scale}\nI Output downscale factor: {downscale}\n'
                f'I Window size: {windowSize}\nI Quality: {quality}\n'
                f'I Calculate confidence: {calcConf}\nI Confidence threshold: {self.conf_threshold}\n'
                f'I Confidence type: {conftype}\nI Uniqueness ratio: {self.uniqueness}\n'
                f'I Penalty P1: {self.p1}\nI Penalty P2: {self.p2}\nI Adaptive P2 alpha: {self.p2_alpha}\n'
                f'I Include diagonals: {includeDiagonals}\nI Number of passes: {numPasses}\n'
                f'I Output mode: {self.output_mode}\nI Verbose: {self.verbose}\n'
                , end='', flush=True)

        left_img  = self.bridge.imgmsg_to_cv2(left_img_msg, "mono16")
        right_img = self.bridge.imgmsg_to_cv2(right_img_msg, "mono16")

        if self.backend == 'pva':
            left_img = cv2.resize(left_img, (480, 270))
            right_img = cv2.resize(right_img, (480, 270))

        if self.backend == 'pva-nvenc-vic':
            left_img = cv2.resize(left_img, (1920, 1080))
            right_img = cv2.resize(right_img, (1920, 1080))
    
        # Streams for left and right independent pre-processing
        streamLeft = vpi.Stream()
        streamRight = vpi.Stream()
    
        # Load input into a vpi.Image and convert it to grayscale, 16bpp
        with vpi.Backend.CUDA:
            with streamLeft:
                left = vpi.asimage(np.asarray(left_img)).convert(vpi.Format.Y16_ER, scale=scale)
            with streamRight:
                right = vpi.asimage(np.asarray(right_img)).convert(vpi.Format.Y16_ER, scale=scale)
    
        # Preprocess input
        # Block linear format is needed for pva-nvenc-vic pipeline and ofa backends
        # Currently we can only convert to block-linear using VIC backend.
        # The input also must be 1080p for pva-nvenc-vic backend.
        if self.backend in {'pva-nvenc-vic', 'ofa-pva-vic', 'ofa'}:
            if self.verbose:
                print(f'W {self.backend} forces to convert input images to block linear', flush=True)
            with vpi.Backend.VIC:
                with streamLeft:
                    left = left.convert(vpi.Format.Y16_ER_BL)
                with streamRight:
                    right = right.convert(vpi.Format.Y16_ER_BL)
            if self.backend == 'pva-nvenc-vic':
                if left.size[0] != 1920 or left.size[1] != 1080:
                    raise ValueError(f'E {self.backend} requires input to be 1920x1080')
    
        if self.verbose:
            print(f'I Input left image: {left.size} {left.format}\n'
                f'I Input right image: {right.size} {right.format}', flush=True)
    
        confidenceU16 = None
    
        if self.backend == 'pva-nvenc-vic':
            if self.verbose:
                print(f'W {self.backend} forces to calculate confidence', flush=True)
            calcConf = True
    
        if calcConf:
            if self.backend not in {'cuda', 'ofa-pva-vic', 'pva-nvenc-vic'}:
                # Only CUDA, OFA-PVA-VIC and PVA-NVENC-VIC have confidence map
                calcConf = False
                if self.verbose:
                    print(f'W {self.backend} does not allow to calculate confidence', flush=True)
    
        if calcConf:
            if self.backend == 'pva-nvenc-vic':
                # PVA-NVENC-VIC only supports 1/4 of the input size
                downscale = 4
                if self.verbose:
                    print(f'W {self.backend} forces downscale to {downscale}', flush=True)
    
        outWidth = (left.size[0] + downscale - 1) // downscale
        outHeight = (left.size[1] + downscale - 1) // downscale
    
        if calcConf:
            confidenceU16 = vpi.Image((outWidth, outHeight), vpi.Format.U16)
    
        # Use stream left to consolidate actual stereo processing
        streamStereo = streamLeft
    
        if self.backend in {'pva', 'cpu'}:
            maxDisparity = 64
            if self.verbose:
                print(f'W {self.backend} forces maxDisparity to {maxDisparity}', flush=True)
        elif self.backend == 'ofa-pva-vic' and maxDisparity not in {128, 256}:
            maxDisparity = 256
            if self.verbose:
                print(f'W {self.backend} forces maxDisparity to {maxDisparity}', flush=True)
    
        if self.verbose:
            if 'ofa' not in self.backend:
                print('W Ignoring P2 alpha and number of passes since not an OFA backend', flush=True)
            if self.backend != 'cuda':
                print('W Ignoring uniqueness since not a CUDA backend', flush=True)
            print('I Estimating stereo disparity ... ', end='', flush=True)
    
        # Estimate stereo disparity.
        with streamStereo, backend:
            disparityS16 = vpi.stereodisp(left, right, downscale=downscale, out_confmap=confidenceU16,
                                        window=windowSize, maxdisp=maxDisparity, confthreshold=self.conf_threshold,
                                        quality=quality, conftype=conftype, mindisp=minDisparity,
                                        p1=self.p1, p2=self.p2, p2alpha=self.p2_alpha, uniqueness=self.uniqueness,
                                        includediagonals=includeDiagonals, numpasses=numPasses)
    
        if self.verbose:
            print('done!\nI Post-processing ... ', end='', flush=True)
    
        # Postprocess results and save them to disk
        with streamStereo, vpi.Backend.CUDA:
            # Some backends outputs disparities in block-linear format, we must convert them to
            # pitch-linear for consistency with other backends.
            if disparityS16.format == vpi.Format.S16_BL:
                disparityS16 = disparityS16.convert(vpi.Format.S16, backend=vpi.Backend.VIC)
    
            # Scale disparity and confidence map so that values like between 0 and 255.
    
            # Disparities are in Q10.5 format, so to map it to float, it gets
            # divided by 32. Then the resulting disparity range, from 0 to
            # stereo.maxDisparity gets mapped to 0-255 for proper output.
            # Copy disparity values back to the CPU.
            disparityU8 = disparityS16.convert(vpi.Format.U8, scale=255.0/(32*maxDisparity)).cpu()
    
            # Apply JET colormap to turn the disparities into color, reddish hues
            # represent objects closer to the camera, blueish are farther away.
            disparityColor = cv2.applyColorMap(disparityU8, cv2.COLORMAP_JET)
    
            # Converts to RGB for output with PIL.
            # disparityColor = cv2.cvtColor(disparityColor, cv2.COLOR_BGR2RGB)
    
            if calcConf:
                confidenceU8 = confidenceU16.convert(vpi.Format.U8, scale=255.0/65535).cpu()
    
                # When pixel confidence is 0, its color in the disparity is black.
                mask = cv2.threshold(confidenceU8, 1, 255, cv2.THRESH_BINARY)[1]
                mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                disparityColor = cv2.bitwise_and(disparityColor, mask)
    

    
        # Save results to disk.'0: color; 1: grayscale; 2: raw binary'

        if self.output_mode == 0:
            # Image.fromarray(disparityColor).save(disparity_fname)
            disparity_img_msg = self.bridge.cv2_to_imgmsg(disparityColor, "bgr8")
            if self.verbose:
                print(f'I Output disparity image: {disparityColor.shape} '
                    f'{disparityColor.dtype}', flush=True)
        elif self.output_mode == 1:
            # Image.fromarray(disparityU8).save(disparity_fname)
            disparity_img_msg = self.bridge.cv2_to_imgmsg(disparityU8, "mono8")
            if self.verbose:
                print(f'I Output disparity image: {disparityU8.shape} '
                    f'{disparityU8.dtype}', flush=True)

        disparity_img_msg.header.stamp = rospy.Time.now()
        self.pub_disp.publish(disparity_img_msg)

        if calcConf:
            # Image.fromarray(confidenceU8).save(confidence_fname)
            confidence_img_msg = self.bridge.cv2_to_imgmsg(confidenceU8, "mono8")
            confidence_img_msg.header.stamp = disparity_img_msg.header.stamp
            self.pub_conf.publish(confidence_img_msg)
            if self.verbose:
                print(f'I Output confidence image: {confidenceU8.shape} '
                        f'{confidenceU8.dtype}', flush=True)


def main():
    rospy.init_node('stereo_image_combiner', anonymous=True)
    processor = StereoImageProcessor()
    rospy.spin()

if __name__ == '__main__':
    main()

