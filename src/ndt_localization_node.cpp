#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "nav2_util/lifecycle_node.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "ndt_localization/ndt_2d.h"

class NDTLocalization : public nav2_util::LifecycleNode
{
public:
  NDTLocalization()
  : nav2_util::LifecycleNode("ndt_localization")
  {
    // Initialize parameters
    declare_parameter("base_frame", "base_link");
    declare_parameter("odom_frame", "odom");
    declare_parameter("global_frame", "map");
    declare_parameter("map_file", "map.pcd");
    declare_parameter("ndt_resolution", 1.0);
    declare_parameter("ndt_step_size", 0.1);
    declare_parameter("ndt_epsilon", 0.01);
    declare_parameter("ndt_max_iterations", 35);
    declare_parameter("voxel_leaf_size", 0.1);
    declare_parameter("use_initial_pose", false);
    declare_parameter("publish_tf", true);
  }

  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    // Convert LaserScan to PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (size_t i = 0; i < scan->ranges.size(); i++) {
      float range = scan->ranges[i];
      if (std::isfinite(range) && range > scan->range_min && range < scan->range_max) {
        float angle = scan->angle_min + i * scan->angle_increment;
        pcl::PointXYZ point;
        point.x = range * std::cos(angle);
        point.y = range * std::sin(angle);
        point.z = 0.0;
        cloud->points.push_back(point);
      }
    }
    
    if (cloud->empty()) {
      RCLCPP_WARN(get_logger(), "Empty point cloud, skipping scan matching");
      return;
    }
    
    // Preprocess scan point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = ndt_.preprocess(cloud, voxel_leaf_size_);
    
    // Set initial guess using current pose estimation
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    if (has_pose_) {
      tf2::Transform transform;
      transform.setOrigin(tf2::Vector3(current_pose_x_, current_pose_y_, 0.0));
      tf2::Quaternion q;
      q.setRPY(0, 0, current_pose_yaw_);
      transform.setRotation(q);
      
      // Convert tf2::Transform to Eigen::Matrix4f
      Eigen::Matrix4f eigen_transform = Eigen::Matrix4f::Identity();
      eigen_transform(0, 0) = transform.getBasis()[0][0];
      eigen_transform(0, 1) = transform.getBasis()[0][1];
      eigen_transform(1, 0) = transform.getBasis()[1][0];
      eigen_transform(1, 1) = transform.getBasis()[1][1];
      eigen_transform(0, 3) = transform.getOrigin().x();
      eigen_transform(1, 3) = transform.getOrigin().y();
      
      init_guess = eigen_transform;
    }
    
    // Set input source and perform NDT alignment
    ndt_.setInputSource(filtered_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    ndt_.align(output_cloud);
    
    if (ndt_.hasConverged()) {
      // Get the transformation matrix
      Eigen::Matrix4f transform_matrix = ndt_.getFinalTransformation();
      
      // Extract position and orientation from transformation matrix
      current_pose_x_ = transform_matrix(0, 3);
      current_pose_y_ = transform_matrix(1, 3);
      current_pose_yaw_ = std::atan2(transform_matrix(1, 0), transform_matrix(0, 0));
      
      // Publish odometry
      auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
      odom_msg->header.stamp = scan->header.stamp;
      odom_msg->header.frame_id = global_frame_;
      odom_msg->child_frame_id = base_frame_;
      
      odom_msg->pose.pose.position.x = current_pose_x_;
      odom_msg->pose.pose.position.y = current_pose_y_;
      odom_msg->pose.pose.position.z = 0.0;
      
      tf2::Quaternion q;
      q.setRPY(0, 0, current_pose_yaw_);
      odom_msg->pose.pose.orientation.x = q.x();
      odom_msg->pose.pose.orientation.y = q.y();
      odom_msg->pose.pose.orientation.z = q.z();
      odom_msg->pose.pose.orientation.w = q.w();
      
      odom_pub_->publish(std::move(odom_msg));
      
      // Broadcast transform
      if (publish_tf_) {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = scan->header.stamp;
        transform_stamped.header.frame_id = global_frame_;
        transform_stamped.child_frame_id = base_frame_;
        
        transform_stamped.transform.translation.x = current_pose_x_;
        transform_stamped.transform.translation.y = current_pose_y_;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform_stamped);
      }
    } else {
      RCLCPP_WARN(get_logger(), "NDT did not converge");
    }
  }

protected:
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    // Get parameters
    base_frame_ = get_parameter("base_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    global_frame_ = get_parameter("global_frame").as_string();
    std::string map_file = get_parameter("map_file").as_string();
    float ndt_resolution = get_parameter("ndt_resolution").as_double();
    float ndt_step_size = get_parameter("ndt_step_size").as_double();
    double ndt_epsilon = get_parameter("ndt_epsilon").as_double();
    int ndt_max_iterations = get_parameter("ndt_max_iterations").as_int();
    voxel_leaf_size_ = get_parameter("voxel_leaf_size").as_double();
    use_initial_pose_ = get_parameter("use_initial_pose").as_bool();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    
    // Configure NDT parameters
    ndt_.setResolution(ndt_resolution);
    ndt_.setStepSize(ndt_step_size);
    ndt_.setTransformationEpsilon(ndt_epsilon);
    ndt_.setMaximumIterations(ndt_max_iterations);
    
    // Load map
    if (!ndt_.loadMap(map_file)) {
      RCLCPP_ERROR(get_logger(), "Failed to load map from %s", map_file.c_str());
      return nav2_util::CallbackReturn::FAILURE;
    }
    
    RCLCPP_INFO(get_logger(), "Map loaded successfully from %s", map_file.c_str());
    
    // Initialize publishers, subscribers, TF
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&NDTLocalization::scanCallback, this, std::placeholders::_1));
    
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    
    // Initial pose subscription
    if (use_initial_pose_) {
      initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10, std::bind(&NDTLocalization::initialPoseCallback, this, std::placeholders::_1));
    }
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    return nav2_util::CallbackReturn::SUCCESS;
  }
  
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Activating");
    
    // Activate publishers
    odom_pub_->on_activate();
    
    return nav2_util::CallbackReturn::SUCCESS;
  }
  
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    
    // Deactivate publishers
    odom_pub_->on_deactivate();
    
    return nav2_util::CallbackReturn::SUCCESS;
  }
  
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    
    // Reset all resources
    scan_sub_.reset();
    odom_pub_.reset();
    initial_pose_sub_.reset();
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();
    
    return nav2_util::CallbackReturn::SUCCESS;
  }
  
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  NDT2D ndt_;
  std::string base_frame_;
  std::string odom_frame_;
  std::string global_frame_;
  double voxel_leaf_size_;
  bool has_pose_ = false;
  bool publish_tf_;
  bool use_initial_pose_;
  double current_pose_x_ = 0.0;
  double current_pose_y_ = 0.0;
  double current_pose_yaw_ = 0.0;

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "Received initial pose");
    
    if (msg->header.frame_id != global_frame_) {
      RCLCPP_WARN(
        get_logger(), "Initial pose not in global frame, got %s but expected %s",
        msg->header.frame_id.c_str(), global_frame_.c_str());
      return;
    }
    
    // Set initial pose
    current_pose_x_ = msg->pose.pose.position.x;
    current_pose_y_ = msg->pose.pose.position.y;
    
    // Extract yaw from quaternion
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pose_yaw_ = yaw;
    
    has_pose_ = true;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NDTLocalization>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
