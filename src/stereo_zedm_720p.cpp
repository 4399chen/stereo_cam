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

    vector<double> D{-0.170127, 0.024090, 0.000306, 0.001427, 0.000000};

    boost::array<double, 9> K = {
        691.14625,   0.     , 640.58134,
          0.     , 691.9053 , 368.06266,
        0.     ,   0.     ,   1.     
    };

    // Mat p = getOptimalNewCameraMatrix(k, d, Size(1280, 720), 0);        // get rectified projection.
    
    boost::array<double, 12> P = {
        684.53549,   0.     , 682.17654,   0.     ,
          0.     , 684.53549, 355.03698,   0.     ,
          0.     ,   0.     ,   1.     ,   0.     
    };

    boost::array<double, 9> r = {
         0.99988796,  0.00163077, -0.0148801 ,
        -0.00161219,  0.99999791,  0.00126058,
         0.01488212, -0.00123645,  0.99988849
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
    // cam.header.stamp = ros::Time::now();
    cam.header.stamp.nsec = 0;
    
    return cam;
}

sensor_msgs::CameraInfo getCameraInfoR(void){        // extract cameraInfo.
    sensor_msgs::CameraInfo cam;

    vector<double> D{-0.168092, 0.019713, -0.000505, 0.000931, 0.000000};

    boost::array<double, 9> K = {
        695.45318,   0.     , 647.13748,
          0.     , 696.22071, 346.57524,
          0.     ,   0.     ,   1.     
    };

    // Mat p = getOptimalNewCameraMatrix(k, d, Size(1280, 720), 0);        // get rectified projection.
    
    boost::array<double, 12> P = {
        684.53549,   0.     , 682.17654, -43.37187,
          0.     , 684.53549, 355.03698,   0.     ,
          0.     ,   0.     ,   1.     ,   0.     
    };

    boost::array<double, 9> r = {
         0.99991002,  0.00120627, -0.01336015,
        -0.00122295,  0.99999848, -0.00124041,
         0.01335863,  0.00125664,  0.99990998
    };

    cam.height =720;
    cam.width = 1280;
    cam.distortion_model = "plumb_bob";
    cam.D = D;
    cam.K = K;
    cam.P = P;
    cam.R = r;
    cam.binning_x = 0;
    cam.binning_y = 0;

    cam.header.frame_id = "right_camera";  //frame_id为camera，也就是相机名字
    // cam.header.stamp = ros::Time::now();
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
 
    // cv::VideoCapture cap(0);
    cv::VideoCapture cap(0 + cv::CAP_V4L2);

    cap.set(cv::CAP_PROP_FRAME_WIDTH,width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,height);
    cap.set(cv::CAP_PROP_FPS,fps);
    cap.set(cv::CAP_PROP_BRIGHTNESS, 8);
    cap.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('Y','U','Y','2'));
    // cap.set(cv::CAP_PROP_FOURCC,cv::VideoWriter::fourcc('M','J','P','G'));

    // 设置曝光时间
    double desired_exposure_time = 1;        // min=1 max=80000 step=1 default=312 value=65536 flags=inactive
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);     // 1: Manual Mode, 3: Aperture Priority Mode
    cap.set(cv::CAP_PROP_EXPOSURE, desired_exposure_time);

    if (!cap.isOpened()) {
        cerr << "Error opening video stream" << endl;
        return -1;
    }

    std::cout<<"开始发布图像数据..."<<std::endl;
    std::cout<<"分辨率:"<<width*0.5<<"*"<<height<<std::endl;    
    std::cout<<"帧率:"<<fps<<"fps"<<std::endl;   

    //设置发布数据的频率
    ros::Rate loop_rate(fps); 

    camera_info_L = getCameraInfoL();
    camera_info_R = getCameraInfoR();
 
    while(n.ok())
    {
        //获取图像
        cap >> stereo_image;

        //分割成左右目，避免使用.clone()
        imLeft = stereo_image(cv::Rect(0, 0, width/2, height));
        imRight = stereo_image(cv::Rect(width/2, 0, width/2, height));

        sensor_msgs::ImagePtr msg_L = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imLeft).toImageMsg();
        sensor_msgs::ImagePtr msg_R = cv_bridge::CvImage(std_msgs::Header(), "bgr8", imRight).toImageMsg();

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
