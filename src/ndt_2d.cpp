#include "ndt_localization/ndt_2d.h"

NDT2D::NDT2D() {
    // Initialize NDT parameters with more permissive values for testing
    ndt.setTransformationEpsilon(0.1);  // Increased for better convergence
    ndt.setStepSize(0.2);              // Increased for faster optimization
    ndt.setResolution(0.5);            // Finer resolution for better matching
    ndt.setMaximumIterations(50);      // More iterations for complex alignments
    
    // Initialize map cloud
    map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
}

// パラメータ付きコンストラクタを実装
NDT2D::NDT2D(double resolution, int max_iterations) {
    // Initialize NDT parameters with user-defined values
    ndt.setTransformationEpsilon(0.1);
    ndt.setStepSize(0.2);
    ndt.setResolution(resolution);
    ndt.setMaximumIterations(max_iterations);
    
    // Initialize map cloud
    map_cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
}

NDT2D::~NDT2D() {
    // Clean up if needed
}

void NDT2D::setInputSource(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    ndt.setInputSource(cloud);
}

void NDT2D::setInputTarget(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    ndt.setInputTarget(cloud);
}

void NDT2D::align(pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
    ndt.align(*output);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NDT2D::preprocess(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double leaf_size) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    
    // Use approximate voxel grid filtering for faster processing
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> avg;
    avg.setInputCloud(cloud);
    avg.setLeafSize(leaf_size, leaf_size, leaf_size);
    avg.filter(*filtered_cloud);
    
    return filtered_cloud;
}

Eigen::Matrix4f NDT2D::getFinalTransformation() {
    return ndt.getFinalTransformation();
}

double NDT2D::getFitnessScore() {
    return ndt.getFitnessScore();
}

bool NDT2D::hasConverged() {
    return ndt.hasConverged();
}

bool NDT2D::loadMap(const std::string& filename) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *map_cloud) == -1) {
        return false;
    }
    setInputTarget(map_cloud);
    return true;
}

// 点群を返すバージョンのloadMap
bool NDT2D::loadMap(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1) {
        return false;
    }
    return true;
}

bool NDT2D::saveMap(const std::string& filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    return pcl::io::savePCDFileBinary(filename, *cloud) != -1;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr NDT2D::getMap() const {
    return map_cloud;
}

// マップグリッドを作成するメソッド
void NDT2D::createMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // グリッドをクリア
    grid_cells.clear();
    
    // 点群が空ならば何もしない
    if (cloud->empty()) {
        return;
    }
    
    // グリッドセルの計算
    // テスト用の簡易実装：点群をいくつかのセルに分ける
    float resolution = ndt.getResolution();
    
    // 点群の境界を計算
    float min_x = std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    
    for (const auto& point : cloud->points) {
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_y, point.y);
    }
    
    // グリッドの次元を計算
    int grid_size_x = static_cast<int>((max_x - min_x) / resolution) + 1;
    int grid_size_y = static_cast<int>((max_y - min_y) / resolution) + 1;
    
    // グリッドセルを初期化
    std::vector<std::vector<std::vector<pcl::PointXYZ>>> grid_points(
        grid_size_x, std::vector<std::vector<pcl::PointXYZ>>(grid_size_y));
    
    // 点をグリッドに割り当てる
    for (const auto& point : cloud->points) {
        int idx_x = static_cast<int>((point.x - min_x) / resolution);
        int idx_y = static_cast<int>((point.y - min_y) / resolution);
        
        if (idx_x >= 0 && idx_x < grid_size_x && idx_y >= 0 && idx_y < grid_size_y) {
            grid_points[idx_x][idx_y].push_back(point);
        }
    }
    
    // 各グリッドセルの統計量を計算
    for (int x = 0; x < grid_size_x; ++x) {
        for (int y = 0; y < grid_size_y; ++y) {
            const auto& points = grid_points[x][y];
            
            if (points.size() < 3) {
                continue; // 点が少なすぎる場合はスキップ
            }
            
            // 平均を計算
            Eigen::Vector2f mean = Eigen::Vector2f::Zero();
            for (const auto& point : points) {
                mean[0] += point.x;
                mean[1] += point.y;
            }
            mean /= static_cast<float>(points.size());
            
            // 共分散を計算
            Eigen::Matrix2f covariance = Eigen::Matrix2f::Zero();
            for (const auto& point : points) {
                Eigen::Vector2f diff;
                diff[0] = point.x - mean[0];
                diff[1] = point.y - mean[1];
                covariance += diff * diff.transpose();
            }
            covariance /= static_cast<float>(points.size() - 1);
            
            // グリッドセルを追加
            GridCell cell;
            cell.mean = mean;
            cell.covariance = covariance;
            cell.point_count = points.size();
            grid_cells.push_back(cell);
        }
    }
    
    // ターゲットとして設定
    setInputTarget(cloud);
    map_cloud = cloud;
}

// マップを更新するメソッド
void NDT2D::updateMap(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // 簡易実装：新しい点群で現在のマップを置き換え
    createMap(cloud);
}

// グリッドサイズを取得するメソッド
size_t NDT2D::getGridSize() const {
    return grid_cells.size();
}

// マッチング（位置推定）を行うメソッド
Eigen::Vector3d NDT2D::match(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, const Eigen::Vector3d& initial_pose) {
    // 初期位置から変換行列を作成
    Eigen::Matrix4f init_guess = Eigen::Matrix4f::Identity();
    
    // 2D位置（x, y, yaw）を変換行列に組み込む
    float cos_yaw = std::cos(initial_pose[2]);
    float sin_yaw = std::sin(initial_pose[2]);
    
    init_guess(0, 0) = cos_yaw;
    init_guess(0, 1) = -sin_yaw;
    init_guess(1, 0) = sin_yaw;
    init_guess(1, 1) = cos_yaw;
    init_guess(0, 3) = initial_pose[0];
    init_guess(1, 3) = initial_pose[1];
    
    // 前処理
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = preprocess(cloud);
    
    // 入力点群として設定
    setInputSource(filtered_cloud);
    
    // アライメントを実行
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());
    ndt.setInputSource(filtered_cloud);
    ndt.align(*aligned, init_guess);
        
    // Check for convergence
    if (!ndt.hasConverged()) {
        std::cerr << "NDT alignment did not converge!" << std::endl;
        return Eigen::Vector3d::Zero(); // Return an invalid/default transformation
    }
   
    // 結果の変換行列から2D位置を抽出
    Eigen::Matrix4f transform = ndt.getFinalTransformation();
    
    double x = transform(0, 3);
    double y = transform(1, 3);
    double yaw = std::atan2(transform(1, 0), transform(0, 0));
    
    return Eigen::Vector3d(x, y, yaw);
}

void NDT2D::setResolution(float resolution) {
    ndt.setResolution(resolution);
}

void NDT2D::setStepSize(float step_size) {
    ndt.setStepSize(step_size);
}

void NDT2D::setMaximumIterations(int max_iterations) {
    ndt.setMaximumIterations(max_iterations);
}

void NDT2D::setTransformationEpsilon(double epsilon) {
    ndt.setTransformationEpsilon(epsilon);
}
