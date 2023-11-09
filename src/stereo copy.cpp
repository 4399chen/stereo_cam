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

    vector<double> D{-0.026991, 0.063441, -0.002903, 0.000359, 0.000000};

    boost::array<double, 9> K = {
		868.661036, 0.000000, 694.207824,
		0.000000, 869.453582, 410.048725,
		0.000000, 0.000000, 1.000000
    };

    // Mat p = getOptimalNewCameraMatrix(k, d, Size(1280, 720), 0);        // get rectified projection.
    
    boost::array<double, 12> P = {
		884.414227, 0.000000, 	691.333285, 0.000000,
		0.000000, 	885.652691, 406.333550, 0.000000,
		0.000000, 	0.000000, 	1.000000, 	0.000000
		
    };

    boost::array<double, 9> r = {
		1.000000, 0.000000, 0.000000,
		0.000000, 1.000000, 0.000000,
		0.000000, 0.000000, 1.000000
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

    vector<double> D{-0.026991, 0.063441, -0.002903, 0.000359, 0.000000};

    boost::array<double, 9> K = {
		868.661036, 0.000000, 694.207824,
		0.000000, 869.453582, 410.048725,
		0.000000, 0.000000, 1.000000
    };

    // Mat p = getOptimalNewCameraMatrix(k, d, Size(1280, 720), 0);        // get rectified projection.
    
    boost::array<double, 12> P = {
		884.414227, 0.000000, 	691.333285, 0.000000,
		0.000000, 	885.652691, 406.333550, 0.000000,
		0.000000, 	0.000000, 	1.000000, 	0.000000
		
    };

    boost::array<double, 9> r = {
		1.000000, 0.000000, 0.000000,
		0.000000, 1.000000, 0.000000,
		0.000000, 0.000000, 1.000000
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
    int fps=30;
 
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
        msg_L->header.stamp=ros::Time::now();
        msg_R->header.stamp=ros::Time::now();
        camera_info_L = getCameraInfoL();
        camera_info_R = getCameraInfoR();
        //发布消息
        PubL.publish(msg_L);
        pubLi.publish(camera_info_L);
        PubR.publish(msg_R);
        pubRi.publish(camera_info_R);
        //按照前面设置的频率将程序挂起
        loop_rate.sleep();
    }
}