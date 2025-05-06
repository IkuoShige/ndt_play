#!/bin/bash

# NDT Localizationテスト実行スクリプト

echo "NDT Localizationテストスイートを実行します..."

# ワークスペースのルートディレクトリに移動
cd $(dirname "$0")/../../..

# ビルドディレクトリの確認
if [ ! -d "build" ]; then
  echo "エラー: ビルドディレクトリが見つかりません。まずcolcon buildを実行してください。"
  exit 1
fi

# テストの実行
echo -e "\n===== 単体テスト =====\n"
echo "スキャンマッチングテスト:"
./build/ndt_localization/test_scan_matching --gtest_color=yes

echo -e "\n点群前処理テスト:"
./build/ndt_localization/test_ndt_preprocessing --gtest_color=yes

echo -e "\nマップ管理テスト:"
./build/ndt_localization/test_map_management --gtest_color=yes

echo -e "\n===== 統合テスト =====\n"
./build/ndt_localization/test_ndt_integration --gtest_color=yes

echo -e "\n===== パフォーマンステスト =====\n"
./build/ndt_localization/test_ndt_performance --gtest_color=yes

echo -e "\nすべてのテストが完了しました。"

# テスト結果の要約
echo -e "\n===== テスト結果の要約 =====\n"
find ./build/ndt_localization -name "*.xml" -exec grep -l "<testsuites>" {} \; | xargs grep -l "<failure" > /dev/null 2>&1
if [ $? -ne 0 ]; then
  echo -e "\033[32mすべてのテストが成功しました！\033[0m"
else
  echo -e "\033[31m失敗したテストがあります。上記のログを確認してください。\033[0m"
  exit 1
fi