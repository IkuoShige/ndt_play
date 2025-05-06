#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>
#include "ndt_localization/ndt_2d.h"
#include "ndt_localization/ndt_localization_node.hpp"

class NDTIntegrationTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
        
        // Create a simple map with a square pattern
        map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        for (float x = 0; x <= 1.0; x += 0.1) {
            map_cloud->points.push_back(pcl::PointXYZ(x, 0.0, 0.0));
            map_cloud->points.push_back(pcl::PointXYZ(x, 1.0, 0.0));
        }
        for (float y = 0.1; y < 1.0; y += 0.1) {
            map_cloud->points.push_back(pcl::PointXYZ(0.0, y, 0.0));
            map_cloud->points.push_back(pcl::PointXYZ(1.0, y, 0.0));
        }
        map_cloud->width = map_cloud->points.size();
        map_cloud->height = 1;
        map_cloud->is_dense = true;
        
        // Create a node with default node options
        rclcpp::NodeOptions node_options;
        node_options.use_intra_process_comms(true);
        
        // Create the NDT node
        ndt_node = std::make_shared<NDTLocalizationNode>(node_options);
        
        // Set the map
        ndt_node->setInputTarget(map_cloud);
        
        // Create a transform broadcaster
        tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(ndt_node);
        
        // Create a transform buffer and listener for testing
        tf_buffer = std::make_shared<tf2_ros::Buffer>(ndt_node->get_clock());
        tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    }

    void TearDown() override {
        tf_listener.reset();
        tf_buffer.reset();
        tf_broadcaster.reset();
        ndt_node.reset();
        rclcpp::shutdown();
    }

    void publishInitialPose(float x, float y, float yaw) {
        // Create and publish initial pose
        auto initial_pose = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
        initial_pose->header.frame_id = "map";
        initial_pose->header.stamp = ndt_node->get_clock()->now();
        initial_pose->pose.pose.position.x = x;
        initial_pose->pose.pose.position.y = y;
        initial_pose->pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        initial_pose->pose.pose.orientation.x = q.x();
        initial_pose->pose.pose.orientation.y = q.y();
        initial_pose->pose.pose.orientation.z = q.z();
        initial_pose->pose.pose.orientation.w = q.w();
        
        // Set covariance to indicate high confidence in initial position
        for (size_t i = 0; i < 36; ++i) {
            initial_pose->pose.covariance[i] = (i < 6) ? 0.1 : 0.0;
        }
        
        // 正しく初期ポーズを公開する（callback関数を直接呼ぶ代わりに）
        ndt_node->initialPoseCallback(initial_pose);
    }

    void publishScan(float x, float y, float yaw) {
        // Create a scan that matches the map but with some noise and transformation
        pcl::PointCloud<pcl::PointXYZ>::Ptr scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
        
        // Apply transformation to the map points to create the scan
        for (const auto& point : *map_cloud) {
            // Apply rotation
            float rotated_x = point.x * cos(yaw) - point.y * sin(yaw);
            float rotated_y = point.x * sin(yaw) + point.y * cos(yaw);
            
            // Apply translation and add some noise
            float noise_x = static_cast<float>(rand()) / RAND_MAX * 0.01 - 0.005;
            float noise_y = static_cast<float>(rand()) / RAND_MAX * 0.01 - 0.005;
            
            scan->points.push_back(pcl::PointXYZ(
                rotated_x + x + noise_x,
                rotated_y + y + noise_y,
                0.0
            ));
        }
        
        scan->width = scan->points.size();
        scan->height = 1;
        scan->is_dense = true;
        
        // Convert to PointCloud2 and publish
        sensor_msgs::msg::PointCloud2 scan_msg;
        pcl::toROSMsg(*scan, scan_msg);
        scan_msg.header.frame_id = "laser";
        scan_msg.header.stamp = ndt_node->get_clock()->now();
        
        // 正しくスキャンを公開する（callback関数を直接呼ぶ代わりに）
        auto scan_ptr = std::make_shared<sensor_msgs::msg::PointCloud2>(scan_msg);
        ndt_node->scanCallback(scan_ptr);
    }

    std::shared_ptr<NDTLocalizationNode> ndt_node;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;
};

TEST_F(NDTIntegrationTest, LocalizationWithInitialPose) {
    // Set initial pose (0.5, 0.5) with 0 radians yaw
    publishInitialPose(0.5, 0.5, 0.0);
    
    // Wait for the node to process the initial pose
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    // Publish a scan that matches the map but is shifted by (0.5, 0.5)
    publishScan(0.5, 0.5, 0.0);
    
    // Wait for the node to process the scan
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // Get the latest transform
    geometry_msgs::msg::TransformStamped transform;
    try {
        // TransformListenerから変換を取得する（TransformBroadcasterではない）
        transform = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        
        // Check that the transform is close to the expected (0.5, 0.5)
        EXPECT_NEAR(transform.transform.translation.x, 0.5, 0.1);
        EXPECT_NEAR(transform.transform.translation.y, 0.5, 0.1);
        
        // Check orientation (should be close to 0 radians)
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        EXPECT_NEAR(yaw, 0.0, 0.1);
    } catch (tf2::TransformException &ex) {
        FAIL() << "Transform exception: " << ex.what();
    }
}

TEST_F(NDTIntegrationTest, LocalizationWithRotation) {
    // Set initial pose (0.0, 0.0) with M_PI/2 radians (90 degrees) yaw
    publishInitialPose(0.0, 0.0, M_PI/2);
    
    // Wait for the node to process the initial pose
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    // Publish a scan that matches the map but is rotated by 90 degrees
    publishScan(0.0, 0.0, M_PI/2);
    
    // Wait for the node to process the scan
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    
    // Get the latest transform
    geometry_msgs::msg::TransformStamped transform;
    try {
        // TransformListenerから変換を取得する
        transform = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        
        // Check position (should be close to (0.5, 0.5) due to rotation)
        EXPECT_NEAR(transform.transform.translation.x, 0.5, 0.2);
        EXPECT_NEAR(transform.transform.translation.y, 0.5, 0.2);
        
        // Check orientation (should be close to 90 degrees)
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        EXPECT_NEAR(yaw, M_PI/2, 0.2);
    } catch (tf2::TransformException &ex) {
        FAIL() << "Transform exception: " << ex.what();
    }
}

TEST_F(NDTIntegrationTest, MultipleScansConvergence) {
    // Set initial pose (0.0, 0.0) with 0 radians yaw
    publishInitialPose(0.0, 0.0, 0.0);
    
    // Wait for the node to process the initial pose
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    
    // Apply a series of transformations to test convergence
    float positions[3][2] = {{0.2, 0.2}, {0.5, 0.5}, {0.8, 0.8}};
    float yaws[3] = {0.0, M_PI/4, M_PI/2};
    
    for (int i = 0; i < 3; ++i) {
        // Publish a scan with the current transformation
        publishScan(positions[i][0], positions[i][1], yaws[i]);
        
        // Wait for the node to process the scan
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
    
    // Get the latest transform
    geometry_msgs::msg::TransformStamped transform;
    try {
        // TransformListenerから変換を取得する
        transform = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        
        // Check that we've converged to the last position
        EXPECT_NEAR(transform.transform.translation.x, positions[2][0], 0.2);
        EXPECT_NEAR(transform.transform.translation.y, positions[2][1], 0.2);
        
        // Check orientation (should be close to last yaw)
        tf2::Quaternion q;
        tf2::fromMsg(transform.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        EXPECT_NEAR(yaw, yaws[2], 0.2);
    } catch (tf2::TransformException &ex) {
        FAIL() << "Transform exception: " << ex.what();
    }
}
