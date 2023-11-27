# stereo-camera
## todo

- 测距
- 测向
- 位姿估计
- 视差图
- depth
- pointcloud
- VIO
- SLAM

## 双目相机的拆分

用opencv读取拼合在一起的双目相机图像拆分成四个ROS Topic

```
/camera/left/camera_info
/camera/left/image_raw
/camera/right/camera_info
/camera/right/image_raw
```
仅可在linux运行

## 双目相机的标定

用的是ROS官方的camera_calibration方法

https://github.com/ros-perception/image_pipeline

```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.043 right:=/camera/right/image_raw left:=/camera/left/image_raw
```

获取到两个相机的四大矩阵

## 双目相机的校正

把获取到的矩阵输入回/camera_info，使用ROS的stereo_image_proc进行双目校正

```
ROS_NAMESPACE=camera rosrun stereo_image_proc stereo_image_proc
```

## launch

```
roslaunch stereo_cam stereo.launch
```

## 深度图

做了两种，基于Opencv的SGBM法，和基于Nvidia VPI的深度图。

VPI参考资料详见：

https://docs.nvidia.com/vpi/getting_started.html

性能测试详见：
https://blog.csdn.net/DCCSDNDC/article/details/134643056?spm=1001.2014.3001.5501
