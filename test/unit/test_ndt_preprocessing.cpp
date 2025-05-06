#include <gtest/gtest.h>
#include <memory>
#include "ndt_localization/ndt_2d.h"

class NDTPreprocessingTest : public ::testing::Test {
protected:
    void SetUp() override {
        ndt = std::make_unique<NDT2D>();
    }

    std::unique_ptr<NDT2D> ndt;
};

TEST_F(NDTPreprocessingTest, DownsamplePointCloud) {
    // Create a test point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    // Add some test points (10x10 grid)
    for (float x = 0; x < 1.0; x += 0.1) {
        for (float y = 0; y < 1.0; y += 0.1) {
            cloud->points.push_back(pcl::PointXYZ(x, y, 0.0));
        }
    }
    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;

    // Check original point cloud properties
    EXPECT_EQ(cloud->points.size(), 100);
    
    // Downsample the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = ndt->preprocess(cloud, 0.2);
    
    // Check that filtering was applied correctly
    EXPECT_LT(filtered_cloud->points.size(), cloud->points.size());
    EXPECT_GT(filtered_cloud->points.size(), 0);
    
    // Check that points are spaced at least 0.2 units apart
    for (size_t i = 1; i < filtered_cloud->points.size(); ++i) {
        float dx = filtered_cloud->points[i].x - filtered_cloud->points[i-1].x;
        float dy = filtered_cloud->points[i].y - filtered_cloud->points[i-1].y;
        float distance = std::sqrt(dx*dx + dy*dy);
        EXPECT_GE(distance, 0.15);  // Allow slight variation due to filtering algorithm
    }
}

TEST_F(NDTPreprocessingTest, EmptyPointCloud) {
    // Create an empty point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    // Downsample the empty cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = ndt->preprocess(cloud, 0.1);
    
    // Check that we get an empty cloud back
    EXPECT_TRUE(filtered_cloud->points.empty());
}

TEST_F(NDTPreprocessingTest, SinglePointPointCloud) {
    // Create a point cloud with a single point
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->points.push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
    cloud->width = 1;
    cloud->height = 1;
    cloud->is_dense = true;
    
    // Downsample the cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = ndt->preprocess(cloud, 0.1);
    
    // Check that we get a single point back
    EXPECT_EQ(filtered_cloud->points.size(), 1);
    EXPECT_FLOAT_EQ(filtered_cloud->points[0].x, 0.0);
    EXPECT_FLOAT_EQ(filtered_cloud->points[0].y, 0.0);
    EXPECT_FLOAT_EQ(filtered_cloud->points[0].z, 0.0);
}
