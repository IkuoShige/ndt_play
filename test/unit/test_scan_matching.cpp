#include <gtest/gtest.h>
#include <memory>
#include <pcl/registration/ndt.h>
#include "ndt_localization/ndt_2d.h"

class ScanMatchingTest : public ::testing::Test {
protected:
    void SetUp() override {
        ndt = std::make_unique<NDT2D>();
        
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
        
        // Set the map
        ndt->setInputTarget(map_cloud);
    }

    std::unique_ptr<NDT2D> ndt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
};

TEST_F(ScanMatchingTest, PerfectAlignment) {
    // Create a scan that exactly matches the map
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(*map_cloud));
    
    // Align the scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
    ndt->setInputSource(scan);
    ndt->align(output);
    
    // Check that alignment was successful and transformation is identity
    EXPECT_TRUE(ndt->hasConverged());
    EXPECT_LT(ndt->getFitnessScore(), 0.1);
    
    Eigen::Matrix4f transformation = ndt->getFinalTransformation();
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (i == j) {
                EXPECT_NEAR(transformation(i, j), 1.0, 1e-4);
            } else {
                EXPECT_NEAR(transformation(i, j), 0.0, 1e-4);
            }
        }
    }
}

TEST_F(ScanMatchingTest, TranslatedScan) {
    // Create a scan that is shifted by (0.5, 0.5)
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : *map_cloud) {
        scan->points.push_back(pcl::PointXYZ(point.x + 0.5, point.y + 0.5, 0.0));
    }
    scan->width = scan->points.size();
    scan->height = 1;
    scan->is_dense = true;
    
    // Align the scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
    ndt->setInputSource(scan);
    ndt->align(output);
    
    // Check that alignment was successful and transformation is correct
    EXPECT_TRUE(ndt->hasConverged());
    EXPECT_LT(ndt->getFitnessScore(), 0.5);  // Allow higher error due to translation
    
    Eigen::Matrix4f transformation = ndt->getFinalTransformation();
    
    // Check translation part (should be approximately -0.5, -0.5)
    EXPECT_NEAR(transformation(0, 3), -0.5, 0.1);
    EXPECT_NEAR(transformation(1, 3), -0.5, 0.1);
    
    // Check rotation part (should be identity)
    EXPECT_NEAR(transformation(0, 0), 1.0, 1e-4);
    EXPECT_NEAR(transformation(1, 1), 1.0, 1e-4);
    EXPECT_NEAR(transformation(2, 2), 1.0, 1e-4);
}

TEST_F(ScanMatchingTest, RotatedScan) {
    // Create a scan that is rotated by 90 degrees
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : *map_cloud) {
        // Rotate 90 degrees around Z-axis (x -> -y, y -> x)
        scan->points.push_back(pcl::PointXYZ(-point.y + 0.5, point.x + 0.5, 0.0));
    }
    scan->width = scan->points.size();
    scan->height = 1;
    scan->is_dense = true;
    
    // Align the scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
    ndt->setInputSource(scan);
    ndt->align(output);
    
    // Check that alignment was successful
    EXPECT_TRUE(ndt->hasConverged());
    EXPECT_LT(ndt->getFitnessScore(), 0.5);
    
    Eigen::Matrix4f transformation = ndt->getFinalTransformation();
    
    // Check rotation part (should be approximately 90 degrees around Z-axis)
    EXPECT_NEAR(transformation(0, 0), 0.0, 0.1);
    EXPECT_NEAR(transformation(0, 1), -1.0, 0.1);
    EXPECT_NEAR(transformation(1, 0), 1.0, 0.1);
    EXPECT_NEAR(transformation(1, 1), 0.0, 0.1);
    
    // Check translation part (should be approximately (0.5, 0.5))
    EXPECT_NEAR(transformation(0, 3), 0.5, 0.1);
    EXPECT_NEAR(transformation(1, 3), 0.5, 0.1);
}

TEST_F(ScanMatchingTest, NoConvergence) {
    // Create a scan with random points that won't converge
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    for (int i = 0; i < 100; ++i) {
        float x = static_cast<float>(rand()) / RAND_MAX;
        float y = static_cast<float>(rand()) / RAND_MAX;
        scan->points.push_back(pcl::PointXYZ(x, y, 0.0));
    }
    scan->width = scan->points.size();
    scan->height = 1;
    scan->is_dense = true;
    
    // Set very few maximum iterations to force non-convergence
    ndt->setMaximumIterations(1);
    
    // Align the scan
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
    ndt->setInputSource(scan);
    ndt->align(output);
    
    // Check that alignment did not converge
    EXPECT_FALSE(ndt->hasConverged());
    EXPECT_GT(ndt->getFitnessScore(), 1.0);  // High fitness score indicates poor match
}
