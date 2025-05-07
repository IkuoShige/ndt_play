#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "ndt_localization/ndt_2d.h"

class MapManagementTest : public ::testing::Test {
protected:
    void SetUp() override {
        ndt = std::make_unique<NDT2D>();
        
        // テスト用の点群マップを作成
        test_map.reset(new pcl::PointCloud<pcl::PointXYZ>());
        test_map->width = 100;
        test_map->height = 1;
        test_map->points.resize(test_map->width * test_map->height);
        
        // 簡単な格子状のマップを作成
        for (int i = 0; i < 10; ++i) {
            for (int j = 0; j < 10; ++j) {
                test_map->points[i * 10 + j].x = static_cast<float>(i);
                test_map->points[i * 10 + j].y = static_cast<float>(j);
                test_map->points[i * 10 + j].z = 0.0f;
            }
        }
        
        // テスト用マップファイル名
        test_map_filename = "/tmp/test_ndt_map.pcd";
    }
    
    void TearDown() override {
        // テスト後に一時ファイルを削除
        std::remove(test_map_filename.c_str());
    }

    std::unique_ptr<NDT2D> ndt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr test_map;
    std::string test_map_filename;
};

TEST_F(MapManagementTest, SaveAndLoadMap) {
    // マップの保存 - 引数順序を修正
    EXPECT_TRUE(ndt->saveMap(test_map_filename, test_map));
    
    // マップのロード
    pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_map(new pcl::PointCloud<pcl::PointXYZ>());
    EXPECT_TRUE(ndt->loadMap(test_map_filename, loaded_map));
    
    // ロードされたマップが元のマップと同じサイズかチェック
    EXPECT_EQ(test_map->width, loaded_map->width);
    EXPECT_EQ(test_map->height, loaded_map->height);
    EXPECT_EQ(test_map->points.size(), loaded_map->points.size());
    
    // ポイントの値が一致するかチェック
    for (size_t i = 0; i < test_map->points.size(); ++i) {
        EXPECT_FLOAT_EQ(test_map->points[i].x, loaded_map->points[i].x);
        EXPECT_FLOAT_EQ(test_map->points[i].y, loaded_map->points[i].y);
        EXPECT_FLOAT_EQ(test_map->points[i].z, loaded_map->points[i].z);
    }
}

TEST_F(MapManagementTest, InvalidFilePath) {
    // 無効なファイルパスでのマップ保存 - 引数順序を修正
    EXPECT_FALSE(ndt->saveMap("/invalid/path/test_map.pcd", test_map));
    
    // 無効なファイルパスからのマップロード
    pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_map(new pcl::PointCloud<pcl::PointXYZ>());
    EXPECT_FALSE(ndt->loadMap("/invalid/path/test_map.pcd", loaded_map));
}

TEST_F(MapManagementTest, EmptyMap) {
    // 空のマップを作成
    pcl::PointCloud<pcl::PointXYZ>::Ptr empty_map(new pcl::PointCloud<pcl::PointXYZ>());
    
    // 空のマップ保存 - 引数順序を修正
    std::string empty_map_filename = "/tmp/empty_ndt_map.pcd";
    EXPECT_TRUE(ndt->saveMap(empty_map_filename, empty_map));
    
    // 空のマップロード
    pcl::PointCloud<pcl::PointXYZ>::Ptr loaded_empty_map(new pcl::PointCloud<pcl::PointXYZ>());
    EXPECT_TRUE(ndt->loadMap(empty_map_filename, loaded_empty_map));
    
    // 空のマップが正しく読み込まれていることを確認
    EXPECT_EQ(0, loaded_empty_map->points.size());
    
    // テスト後に一時ファイルを削除
    std::remove(empty_map_filename.c_str());
}

TEST_F(MapManagementTest, MapGridCreation) {
    // NDTマップグリッドの作成
    ndt->createMap(test_map);
    
    // グリッドサイズが正しく設定されていることを確認
    EXPECT_GT(ndt->getGridSize(), 0);
    
    // グリッドセルが正しく設定されていることを確認
    // グリッドセルの数は、ポイント数よりも少ないはず（複数のポイントが同じセルに入るため）
    EXPECT_LE(ndt->getGridSize(), test_map->points.size());
}

TEST_F(MapManagementTest, MapUpdateTest) {
    // 初期マップの作成
    ndt->createMap(test_map);
    
    // 初期グリッドサイズを保存
    size_t initial_grid_size = ndt->getGridSize();
    
    // 新しいポイントを追加したマップを作成
    pcl::PointCloud<pcl::PointXYZ>::Ptr updated_map(new pcl::PointCloud<pcl::PointXYZ>(*test_map));
    updated_map->points.push_back(pcl::PointXYZ(20.0f, 20.0f, 0.0f));  // 既存の範囲外の点を追加
    updated_map->width = updated_map->points.size();
    
    // マップの更新
    ndt->updateMap(updated_map);
    
    // グリッドサイズが増加していることを確認
    EXPECT_GT(ndt->getGridSize(), initial_grid_size);
}
