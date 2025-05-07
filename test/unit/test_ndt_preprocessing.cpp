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
    
    // Use a k-d tree to verify spatial proximity
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(filtered_cloud);
    for (size_t i = 0; i < filtered_cloud->points.size(); ++i) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        // Search for neighbors within a radius of 0.15
        if (kdtree.radiusSearch(filtered_cloud->points[i], 0.15, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 1) {
            ADD_FAILURE() << "Point " << i << " has neighbors closer than 0.15 units.";
        }
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
