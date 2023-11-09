#include <iostream>
#include <fstream>
//io代表输入输出，manip是manipulator（操纵器）的缩写,主要是对cin,cout之类的一些操纵运算子
#include <iomanip>
//C++的标准模版库（STL）中最重要的头文件之一，提供了大量基于迭代器的非成员模板函数
#include <algorithm>
//处理日期和时间的库
#include <chrono>
//包含了使用ROS节点的必要文件
#include <ros/ros.h> 
//使用这个读取一张照片，然后转化为ROS的消息格式
#include <opencv2/opencv.hpp> 
//使用这个进行图像格式的转换，将Opencv的Mat转化为ROS的消息格式，或者将ROS格式的信息转化为Opencv的Mat
#include <cv_bridge/cv_bridge.h>  
//包括想要发布或者订阅一个图像的所有东西
#include <image_transport/image_transport.h> 

typedef ::std_msgs::Header_<std::allocator<void> > Header;
using namespace std;
using namespace cv;

sensor_msgs::CameraInfo getCameraInfoL(void){        // extract cameraInfo.
    sensor_msgs::CameraInfo cam;

    vector<double> D{-0.020917, 0.048395, -0.002389, 0.000589, 0.000000};

    boost::array<double, 9> K = {
        867.011713, 0.000000, 695.941232,
        0.000000, 867.816427, 412.125027,
        0.000000, 0.000000, 1.000000
    };

    // Mat p = getOptimalNewCameraMatrix(k, d, Size(1280, 720), 0);        // get rectified projection.
    
    boost::array<double, 12> P = {
        960.583503, 0.000000, 732.428703, 0.000000,
        0.000000, 960.583503, 426.645927, 0.000000,
        0.000000, 0.000000, 1.000000, 0.000000
		
    };

    boost::array<double, 9> r = {
        0.999733, 0.003436, -0.022859,
        -0.003437, 0.999994, -0.000029,
        0.022859, 0.000108, 0.999739
	};

    cam.height = 720;
    cam.width = 1280;
    cam.distortion_model = "plumb_bob";
    cam.D = D;
    cam.K = K;
    cam.P = P;
    cam.R = r;
    cam.binning_x = 0;
    cam.binning_y = 0;

    cam.header.frame_id = "left_camera";  //frame_id为camera，也就是相机名字
    cam.header.stamp = ros::Time::now();
    cam.header.stamp.nsec = 0;
    
    return cam;
}

 
sensor_msgs::CameraInfo getCameraInfoR(void){        // extract cameraInfo.
    sensor_msgs::CameraInfo cam;

    vector<double> D{0.009107, 0.006306, 0.000341, 0.001296, 0.000000};

    boost::array<double, 9> K = {
        863.730103, 0.000000, 707.376905,
        0.000000, 863.973069, 445.883640,
        0.000000, 0.000000, 1.000000
    };

    // Mat p = getOptimalNewCameraMatrix(k, d, Size(1280, 720), 0);        // get rectified projection.
    
    boost::array<double, 12> P = {
        960.583503, 0.000000, 732.428703, -133.457400,
        0.000000, 960.583503, 426.645927, 0.000000,
        0.000000, 0.000000, 1.000000, 0.000000
		
    };

    boost::array<double, 9> r = {
        0.999692, 0.004935, -0.024333,
        -0.004933, 0.999988, 0.000129,
        0.024334, -0.000009, 0.999704
	};

    cam.height = 720;
    cam.width = 1280;
    cam.distortion_model = "plumb_bob";
    cam.D = D;
    cam.K = K;
    cam.P = P;
    cam.R = r;
    cam.binning_x = 0;
    cam.binning_y = 0;

    cam.header.frame_id = "right_camera";  //frame_id为camera，也就是相机名字
    cam.header.stamp = ros::Time::now();
    cam.header.stamp.nsec = 0;
    
    return cam;
}


int main(int argc,char** argv)
{
    //初始化ROS，节点命名为image_publisher，节点命名必须保持唯一
    ros::init(argc,argv,"stereo_publisher");
    //实例化节点，节点进程句柄
    ros::NodeHandle n;
    //初始化
    image_transport::ImageTransport it(n);
    //发布话题
    image_transport::Publisher PubL=it.advertise("/camera/left/image_raw" ,1);
    image_transport::Publisher PubR=it.advertise("/camera/right/image_raw",1);
    ros::Publisher pubLi = n.advertise<sensor_msgs::CameraInfo>("/camera/left/camera_info",  1);
    ros::Publisher pubRi = n.advertise<sensor_msgs::CameraInfo>("/camera/right/camera_info", 1);

    sensor_msgs::CameraInfo camera_info_L;
    sensor_msgs::CameraInfo camera_info_R;


    
    cv::Mat stereo_image,imLeft,imRight;
    int width=2560;
    int height=720;
    int fps=60;
 
    cv::VideoCapture cap(0);
    //分辨率
    cap.set(cv::CAP_PROP_FRAME_WIDTH,width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,height);
    //帧率
    cap.set(cv::CAP_PROP_FPS,fps);
    //视频流格式
    cap.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('Y','U','Y','2'));
 
    std::cout<<"开始发布图像数据..."<<std::endl;
    std::cout<<"分辨率:"<<width*0.5<<"*"<<height<<std::endl;    
    std::cout<<"帧率:"<<fps<<"fps"<<std::endl;    
 
    while(n.ok())
    {
        //获取图像
        cap >> stereo_image;
        //分割成左右目
        /*
        rowRange为指定的行span创建一个新的矩阵头，可取指定行区间元素
        colRange为指定的列span创建一个新的矩阵头，可取指定列区间元素
        分割图像是取宽的一半，所以指定0-width/2的列区间
        不clone会对元矩阵直接操作
        */
        imLeft=stereo_image.colRange(0,width/2).clone();
        imRight=stereo_image.colRange(width/2,width).clone();
        //转成灰度图
        // cv::cvtColor(imLeft,imLeft,cv::COLOR_BGR2GRAY);
        // cv::cvtColor(imRight,imRight,cv::COLOR_BGR2GRAY);
        //将图像类型转化成ROS消息类型
        // sensor_msgs::ImagePtr msg_L = cv_bridge::CvImage(std_msgs::Header(), "mono8", imLeft).toImageMsg();
        // sensor_msgs::ImagePtr msg_R = cv_bridge::CvImage(std_msgs::Header(), "mono8", imRight).toImageMsg();
        sensor_msgs::ImagePtr msg_L = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imLeft).toImageMsg();
        sensor_msgs::ImagePtr msg_R = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imRight).toImageMsg();
        //设置发布数据的频率
        ros::Rate loop_rate(fps);
        //读取当前时间作为图像的时间戳
        camera_info_L = getCameraInfoL();
        camera_info_R = getCameraInfoR();

        camera_info_L.header.stamp = ros::Time::now();
        camera_info_R.header.stamp = camera_info_L.header.stamp;
        msg_L->header.stamp=camera_info_L.header.stamp;
        msg_R->header.stamp=camera_info_L.header.stamp;

        //发布消息
        PubL.publish(msg_L);
        pubLi.publish(camera_info_L);
        PubR.publish(msg_R);
        pubRi.publish(camera_info_R);
        //按照前面设置的频率将程序挂起
        loop_rate.sleep();
    }
}