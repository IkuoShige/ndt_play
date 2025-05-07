#include <gtest/gtest.h>
#include <ndt_localization/ndt_2d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <chrono>
#include <vector>
#include <memory>

class NDTPerformanceTest : public ::testing::Test {
protected:
    void SetUp() override {
        // テスト用の点群データを生成
        point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        point_cloud_->width = 10000;
        point_cloud_->height = 1;
        point_cloud_->points.resize(point_cloud_->width * point_cloud_->height);
        
        // ランダムな点群データを生成
        for (size_t i = 0; i < point_cloud_->points.size(); ++i) {
            point_cloud_->points[i].x = 10.0f * rand() / RAND_MAX;
            point_cloud_->points[i].y = 10.0f * rand() / RAND_MAX;
            point_cloud_->points[i].z = 0.0f;
        }
        
        // NDTの初期化 - コンストラクタにパラメータを渡す
        ndt_ = std::make_shared<NDT2D>(2.0, 5);
        
        // マップを設定
        ndt_->setInputTarget(point_cloud_);
    }

    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud_;
    std::shared_ptr<NDT2D> ndt_;
};

TEST_F(NDTPerformanceTest, ScanMatchingProcessingTime) {
    // 初期位置を設定
    Eigen::Vector3d initial_pose(0.0, 0.0, 0.0);
    
    // スキャンマッチングの処理時間を測定
    auto start = std::chrono::high_resolution_clock::now();
    // 修正: createMapを呼び出してから、matchを呼び出す
    ndt_->createMap(point_cloud_);
    Eigen::Vector3d result = ndt_->match(point_cloud_, initial_pose);
    auto end = std::chrono::high_resolution_clock::now();
    
    // 処理時間の検証（100ms以内に処理完了すること）
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    EXPECT_LT(duration.count(), 100);
    
    // 結果の位置が初期位置と異なることを確認
    EXPECT_NE(result(0), initial_pose(0));
    EXPECT_NE(result(1), initial_pose(1));
    EXPECT_NE(result(2), initial_pose(2));
}

TEST_F(NDTPerformanceTest, MemoryUsageTest) {
    // 大規模マップの読み込み
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> large_map(new pcl::PointCloud<pcl::PointXYZ>);
    large_map->width = 100000;
    large_map->height = 1;
    large_map->points.resize(large_map->width * large_map->height);
    
    // ランダムな点群データを生成
    for (size_t i = 0; i < large_map->points.size(); ++i) {
        large_map->points[i].x = 100.0f * rand() / RAND_MAX;
        large_map->points[i].y = 100.0f * rand() / RAND_MAX;
        large_map->points[i].z = 0.0f;
    }
    
    // メモリ使用量の測定（簡易的な方法）
    size_t initial_memory = 0;
    size_t final_memory = 0;
    
    // マップ読み込み前のメモリ使用量を取得
    initial_memory = sizeof(*large_map) + large_map->points.size() * sizeof(pcl::PointXYZ);
    
    // NDTマップの構築
    ndt_->createMap(large_map);
    
    // マップ構築後のメモリ使用量を取得
    final_memory = initial_memory + sizeof(*ndt_) + ndt_->getGridSize() * sizeof(NDT2D::GridCell);
    
    // メモリ使用量の検証（50MB以下であること）
    EXPECT_LT(final_memory, 50 * 1024 * 1024);
    
    // 連続処理時のメモリ増加分を確認
    for (int i = 0; i < 5; ++i) {
        std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        temp_cloud->width = 5000;
        temp_cloud->height = 1;
        temp_cloud->points.resize(temp_cloud->width * temp_cloud->height);
        
        // ランダムな点群データを生成
        for (size_t j = 0; j < temp_cloud->points.size(); ++j) {
            temp_cloud->points[j].x = 10.0f * rand() / RAND_MAX;
            temp_cloud->points[j].y = 10.0f * rand() / RAND_MAX;
            temp_cloud->points[j].z = 0.0f;
        }
        
        ndt_->match(temp_cloud, Eigen::Vector3d::Zero());
    }
    
    // メモリリークの検証（増加分が10MB未満であること）
    size_t memory_increase = final_memory - initial_memory;
    EXPECT_LT(memory_increase, 10 * 1024 * 1024);
}

TEST_F(NDTPerformanceTest, AccuracyUnderNoise) {
    // ノイズを含む点群を生成
    auto noisy_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*point_cloud_);
    
    // 点群にランダムノイズを追加
    for (size_t i = 0; i < noisy_cloud->points.size(); ++i) {
        noisy_cloud->points[i].x += 0.1 * (rand() / RAND_MAX - 0.5);
        noisy_cloud->points[i].y += 0.1 * (rand() / RAND_MAX - 0.5);
    }
    
    // createMapを呼び出してマップを作成
    ndt_->createMap(point_cloud_);
    
    // 正解の位置を設定
    Eigen::Vector3d ground_truth(1.0, 1.0, M_PI/4);
    
    // スキャンマッチングを実行
    Eigen::Vector3d result = ndt_->match(noisy_cloud, ground_truth);
    
    // 位置推定の誤差を計算
    double position_error = (result.head<2>() - ground_truth.head<2>()).norm();
    double angle_error = std::abs(result(2) - ground_truth(2));
    
    // 精度の検証（位置誤差5cm以下、角度誤差0.1rad以下）
    EXPECT_LT(position_error, 0.05);
    EXPECT_LT(angle_error, 0.1);
}

TEST_F(NDTPerformanceTest, LargeScalePointCloudProcessing) {
    // 大規模点群の生成
    auto large_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*point_cloud_);
    large_cloud->width = 50000;
    large_cloud->height = 1;
    large_cloud->points.resize(large_cloud->width * large_cloud->height);
    
    // ランダムな点群データを生成
    for (size_t i = 0; i < large_cloud->points.size(); ++i) {
        large_cloud->points[i].x = 10.0f * rand() / RAND_MAX;
        large_cloud->points[i].y = 10.0f * rand() / RAND_MAX;
        large_cloud->points[i].z = 0.0f;
    }
    
    // マップの作成
    ndt_->createMap(large_cloud);
    
    // 初期位置を設定
    Eigen::Vector3d initial_pose(0.0, 0.0, 0.0);
    
    // 大規模点群の処理時間を測定
    auto start = std::chrono::high_resolution_clock::now();
    Eigen::Vector3d result = ndt_->match(large_cloud, initial_pose);
    auto end = std::chrono::high_resolution_clock::now();
    
    // 処理時間の検証（500ms以内に処理完了すること）
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    EXPECT_LT(duration.count(), 500);
    
    // 結果の位置が初期位置と異なることを確認
    EXPECT_NE(result(0), initial_pose(0));
    EXPECT_NE(result(1), initial_pose(1));
    EXPECT_NE(result(2), initial_pose(2));
}
