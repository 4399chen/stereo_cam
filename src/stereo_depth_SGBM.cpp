#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class StereoDepthNode
{
public:
    StereoDepthNode()
    {
        // 初始化ROS节点
        ros::NodeHandle nh;

        // 使用 message_filters 订阅并同步左右图像
        left_sub.subscribe(nh, "/camera/left/image_rect", 1);
        right_sub.subscribe(nh, "/camera/right/image_rect", 1);

        // 使用ApproximateTime策略同步消息
        sync_.reset(new Sync(mysyncpolicy(1), left_sub, right_sub));

        sync_->registerCallback(boost::bind(&StereoDepthNode::imageCallback, this, _1, _2));
        
        // 发布深度图的话题
        depth_pub = nh.advertise<sensor_msgs::Image>("/stereo/depth/image", 1);

        // 初始化 StereoSGBM 对象
        int minDisparity = 0;
        int numDisparities = 16 * 5; // 16的倍数
        int blockSize = 5; // 推荐的起始值
        int P1 = 8 * blockSize * blockSize;
        int P2 = 32 * blockSize * blockSize;
        int disp12MaxDiff = 1;
        int preFilterCap = 63;
        int uniquenessRatio = 15;
        int speckleWindowSize = 100;
        int speckleRange = 32;
        int mode = cv::StereoSGBM::MODE_SGBM_3WAY;
        // int mode = cv::StereoSGBM::MODE_SGBM;

        sgbm = cv::StereoSGBM::create(minDisparity, numDisparities, blockSize, 
                                      P1, P2, disp12MaxDiff, preFilterCap, 
                                      uniquenessRatio, speckleWindowSize, speckleRange, mode);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(left_msg, sensor_msgs::image_encodings::MONO8);
            left_image = cv_ptr->image;
            left_stamp = left_msg->header.stamp;

            cv_ptr = cv_bridge::toCvCopy(right_msg, sensor_msgs::image_encodings::MONO8);
            right_image = cv_ptr->image;
            right_stamp = right_msg->header.stamp;

            processImages();
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    void processImages()
    {
        if (!left_image.empty() && !right_image.empty())
        {
            // 检查两个图像的时间戳是否足够接近
            if (abs((left_stamp - right_stamp).toSec()) < 0.05) // 50毫秒以内认为是同步的
            {
                // cv::Mat depth_image;
                cv::Mat depth_image = cv::Mat::zeros(left_image.cols, left_image.rows, CV_8UC1);
                sgbm->compute(left_image, right_image, depth_image);

                // 转换深度图格式，如果需要
                cv::Mat normalized_depth;
                cv::normalize(depth_image, normalized_depth, 0, 255,cv::NORM_MINMAX);
                cv::Mat displayable_depth;
                normalized_depth.convertTo(displayable_depth, CV_8U);

                sensor_msgs::ImagePtr depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", displayable_depth).toImageMsg();
                depth_pub.publish(depth_msg);
            }
            else
            {
                ROS_WARN("Image timestamps are not synchronized. Difference: %f seconds", 
                         abs((left_stamp - right_stamp).toSec()));
            }
        }
    }

private:
    ros::Publisher depth_pub;
    cv::Mat left_image, right_image;
    cv::Ptr<cv::StereoSGBM> sgbm;
    ros::Time left_stamp, right_stamp;
    message_filters::Subscriber<sensor_msgs::Image> left_sub;
    message_filters::Subscriber<sensor_msgs::Image> right_sub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> mysyncpolicy;//时间戳对齐规则
    typedef message_filters::Synchronizer<mysyncpolicy> Sync;
    boost::shared_ptr<Sync> sync_;//时间同步器
};

int main(int argc, char** argv)
{
    std::cout << "OpenCV version : " << CV_VERSION << std::endl;
    ros::init(argc, argv, "stereo_depth_node");
    StereoDepthNode node;
    ros::spin();
    return 0;
}
