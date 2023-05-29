#include "cluster.hpp"
// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace nvinfer1;


ClusterNode::ClusterNode()
: Node("clusternode")
{
 loadParams();

  // Cluster
  this->cluster_manager_ = std::make_shared<ClusterManager>(static_cast<int>(this->node_index_), this->number_of_nodes_, this->interval_);

  // Inference
  this->dummy_inference_ = std::make_shared<Yolov7>(this->inference_model_path_);

  // Can Sender
  this->pcan_sender_ = std::make_shared<ObjectDetectionsSender>(this->can_id_, this->time_interval_);

  // QoS
  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
  rclcpp::QoS system_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());


    using std::placeholders::_1;
    if (this->use_compressed_subscriber_)
    {
        this->compressed_raw_image_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("camera/image/compressed", QOS_RKL10V, std::bind(&ClusterNode::compressed_image_callback, this, _1));
    }
    else 
    {
       // this->raw_image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("cluster/image", QOS_RKL10V, std::bind(&Yolov7::image_callback, this, _1));
    }
  //TODO: ADD benchmark
}

ClusterNode::~ClusterNode()
{
    //TODO: ADD benchmark
}



void ClusterNode::loadParams()
{
  // Cluster
  node_index_ = this->declare_parameter("node_index", 0);
  number_of_nodes_ = this->declare_parameter("number_of_nodes", 1);
  interval_ = this->declare_parameter("interval", 10);

  // Inference
  inference_model_path_ = this->declare_parameter("inference_model_path", "/home/avees/ros2_ws/weights/yolov7.engine");

  // Pcan
  use_can_ = this->declare_parameter("use_can", true);
  can_id_ = this->declare_parameter("can_id", 101);
  time_interval_ = this->declare_parameter("time_interval", 500);

  RCLCPP_INFO(this->get_logger(), "Parameters loaded");
}



void ClusterNode::compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr compressed_image)
{
  
   rclcpp::Time ros_time = this->get_clock()->now();
  

    // Convert Ros2 image to OpenCV image
    cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvCopy(compressed_image, sensor_msgs::image_encodings::BAYER_RGGB8);
    if (cv_image->image.empty()) return;

    cv::Mat color_image;
    cv::cvtColor(cv_image->image, color_image, cv::COLOR_BayerRG2RGB);

    image_batch.push_back(color_image);

    // Cluster
    if (this->cluster_manager_->is_self_order(rclcpp::Time(compressed_image->header.stamp).seconds()) == false) return;

   

    // Inference
    std::vector<ObjectDetection> detections = this->dummy_inference_->get_detections(color_image);

    

    if (use_can_)
    {
      // Can send
      this->pcan_sender_->WriteMessages(rclcpp::Time(compressed_image->header.stamp).seconds(), detections);
    }
    

    RCLCPP_INFO(this->get_logger(), "Publish.");


}




