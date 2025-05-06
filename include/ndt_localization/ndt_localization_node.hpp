#ifndef NDT_LOCALIZATION_NODE_HPP
#define NDT_LOCALIZATION_NODE_HPP

#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "ndt_localization/ndt_2d.h"
#include "pcl_conversions/pcl_conversions.h"

class NDTLocalizationNode : public rclcpp::Node
{
public:
    NDTLocalizationNode(const rclcpp::NodeOptions & options)
    : Node("ndt_localization", options),
      ndt_(std::make_unique<NDT2D>())
    {
        // 初期化処理をここに追加
        scan_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "scan", 10, std::bind(&NDTLocalizationNode::scanCallback, this, std::placeholders::_1));
        
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&NDTLocalizationNode::initialPoseCallback, this, std::placeholders::_1));
    }

    // テスト用にパブリックにする
    void scanCallback(const sensor_msgs::msg::PointCloud2::SharedPtr scan)
    {
        // 点群処理
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // 初期ポーズ処理
    }

    void setInputTarget(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
    {
        ndt_->setInputTarget(cloud);
    }

    // テスト用に公開するメンバ
    std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>> scan_sub_;
    std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>> initial_pose_sub_;

private:
    std::unique_ptr<NDT2D> ndt_;
};

#endif // NDT_LOCALIZATION_NODE_HPP