#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


class ImageProcessor
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageProcessor()
        : it_(nh_)
    {
        // 订阅下位机发布的Astra Pro图像话题
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ImageProcessor::imageCallback, this);
        
        // 发布加速后的图像话题
        image_pub_ = it_.advertise("/camera/rgb2gray/image_speedup", 1);

        // 初始化视频流
        VideoCapture cap("http://192.168.199.191:9527/camera/");
        
        if (!cap.isOpened())
        {
            ROS_ERROR("Failed to open video stream!");
        }

        // 循环读取视频帧并加速
        Mat frame;
        while (nh_.ok())
        {
            cap >> frame;
            if (frame.empty())
            {
                ROS_ERROR("Failed to read frame from video stream!");
                break;
            }

            // 加速视频帧
            Mat grayFrame;
            cvtColor(frame, grayFrame, COLOR_RGB2GRAY);
            resize(grayFrame, grayFrame, Size(640, 480), 0,0, INTER_LINEAR);
            //cv::resize(frame, frame, cv::Size(640, 480), 0, 0, cv::INTER_LINEAR);
            Mat dst;
            GaussianBlur(frame, dst, Size(0, 0), 3);
            addWeighted(frame, 1.5, dst, -0.5, 0, dst);

            // 发布加速后的图像
            sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
            image_pub_.publish(msg);

            // 等待10毫秒
            waitKey(10);
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            // 将ROS图像消息转换为OpenCV图像
            Mat img = cv_bridge::toCvShare(msg, "bgr8")->image;

            // 处理图像
            Mat dst;
            GaussianBlur(img, dst, cv::Size(0, 0), 3);
            addWeighted(img, 1.5, dst, -0.5, 0, dst);

            // 将OpenCV图像转换为ROS图像消息并发布
            sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
            image_pub_.publish(msg_out);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_speedup");
    ImageProcessor ip;
    ros::spin();
    return 0;
}
