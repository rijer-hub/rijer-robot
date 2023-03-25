#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudaarithm.hpp>

using namespace cv;

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  try
  {
    // 将ROS图像消息转换为OpenCV图像格式
    cv::Mat depth_image = cv_bridge::toCvCopy(msg, "16UC1")->image;

    // 将深度图像数据传输到GPU
    cv::cuda::GpuMat depth_image_gpu(depth_image);

    // 创建一个GpuMat对象，用于存储加速后的深度图像
    cv::cuda::GpuMat depth_image_gpu_result;

    // 使用CUDA加速，将深度图像的所有像素值乘以2
    cv::cuda::multiply(depth_image_gpu, 2, depth_image_gpu_result);

    // 将加速后的深度图像数据传输回CPU
    cv::Mat depth_image_result;
    depth_image_gpu_result.download(depth_image_result);

    // 显示处理后的图像
    cv::imshow("Depth Image", depth_image_result);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to '16UC1'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_image_viewer_cuda");
  ros::NodeHandle nh;

  // 订阅下位机发布的深度图像数据
  ros::Subscriber sub = nh.subscribe("/camera/depth/image_raw", 1, imageCallback);

  ros::spin();
  
  return 0;
}
