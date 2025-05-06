#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <string>
#include <vector>

class NDT2D {
public:
    // GridCellクラスを追加
    struct GridCell {
        Eigen::Vector2f mean;
        Eigen::Matrix2f covariance;
        int point_count;
    };

    NDT2D();
    // パラメータ付きコンストラクタを追加
    NDT2D(double resolution, int max_iterations);
    ~NDT2D();

    // Core NDT functionality
    void setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void align(pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    
    // Point cloud preprocessing
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double leaf_size = 0.1);
    
    // Position estimation
    Eigen::Matrix4f getFinalTransformation();
    double getFitnessScore();
    bool hasConverged();
    
    // Map management
    bool loadMap(const std::string& filename);
    // 点群を返すバージョンのloadMapを追加
    bool loadMap(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    bool saveMap(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr getMap() const;
    
    // テスト用に追加するメソッド
    // マップグリッドを作成
    void createMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    // マップを更新
    void updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    // グリッドサイズを取得
    size_t getGridSize() const;
    // マッチング（位置推定）を行う
    Eigen::Vector3d match(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3d& initial_pose);
    
    // Parameter setters
    void setResolution(float resolution);
    void setStepSize(float step_size);
    void setMaximumIterations(int max_iterations);
    void setTransformationEpsilon(double epsilon);

private:
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;
    std::vector<GridCell> grid_cells; // グリッドセルを格納
};
