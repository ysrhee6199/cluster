#include <chrono>
#include <thread>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <fstream>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>



// Cluster
#include "cluster_inference/cluster_inference.hpp"

// Inference
#include "dummy_inference/yolov7.hpp"

// Pcan Senser
#include "pcan/ObjectDetectionsSender.hpp"

// benchmark
#include <string>
#include <ctime>
#include <fstream>


using namespace nvinfer1;



class ClusterNode : public rclcpp::Node
{
public:
  ClusterNode();
  ~ClusterNode();

private:
   // Cluster
  std::shared_ptr<ClusterManager> cluster_manager_;

  // Inference
  std::shared_ptr<Yolov7> dummy_inference_;

  // Pcan Senser
  std::shared_ptr<ObjectDetectionsSender> pcan_sender_;

  // parameter
  int32_t node_index_;
  int number_of_nodes_;
  int interval_;
  bool use_compressed_subscriber_ = true;
  std::string inference_model_path_;

  bool use_can_;
  unsigned int can_id_;
  int time_interval_;





  void loadParams();


  void image_callback(const sensor_msgs::msg::Image::SharedPtr image);
  void initialize_model();
 
  // use sensor_msgs::msg::CompressedImage
  void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr compressed_image);


  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_image_subscriber_;

  
};

