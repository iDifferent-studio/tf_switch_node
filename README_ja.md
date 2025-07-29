# tf_switch_node

[![GitHub](https://img.shields.io/badge/GitHub-iDifferent--studio%2Ftf__switch__node-blue)](https://github.com/iDifferent-studio/tf_switch_node)

**言語:** [English](README.md) | 日本語

## 概要
`tf_switch_node`は、状態機械（ステートマシン）アプローチを使用して、EKF（拡張カルマンフィルタ）とAMCL（適応モンテカルロ定位）の間で`map->odom`変換のソースを切り替えるROS 2パッケージです。このノードはAMCLポーズの共分散とGPS fix状態を監視し、明確に定義された状態遷移を通じてどちらの定位手法が`map->odom`変換を配信すべきかを決定し、異なる環境において信頼性の高い定位性能を確保します。

## 機能
- **状態機械ベースの変換切り替え**: 決定論的な状態機械を使用してEKFまたはAMCLのどちらが`map->odom`変換を配信すべきかを決定
- **AMCLポーズ監視**: AMCLポーズの共分散を継続的に監視して定位品質を評価
- **GPS状態統合**: `/fix`トピックからGPS fix状態を評価してGPSの信頼性を判断
- **デュアル定位サポート**: EKFベースとAMCLベースの定位手法間でシームレスに切り替え
- **決定論的意思決定**: 事前定義された状態機械ロジックとセンサーデータ閾値に基づいて切り替え判定を実行
- **ROS 2ネイティブ**: モダンなC++プラクティスでROS 2専用に構築

## インストール
`tf_switch_node`パッケージをビルドしてインストールするには、以下の手順に従ってください：

1. **ROS 2ワークスペースの作成：**
   ```bash
   mkdir -p ~/tf_switch_ws/src
   cd ~/tf_switch_ws/src
   ```

2. **リポジトリのクローン：**
   ```bash
   git clone https://github.com/iDifferent-studio/tf_switch_node.git
   cd ~/tf_switch_ws
   ```

3. **依存関係のインストール：**
   必要な依存関係がすべてインストールされていることを確認してください。以下のコマンドを使用できます：
   ```bash
   rosdep install -i --from-path src --rosdistro humble -y
   ```

4. **パッケージのビルド：**
   以下のコマンドを使用してパッケージをビルドします：
   ```bash
   colcon build --packages-select tf_switch_node
   ```

5. **セットアップファイルのソース：**
   ビルド後、セットアップファイルをソースします：
   ```bash
   source install/setup.bash
   ```

## 使用方法
`tf_switch_node`を実行するには、以下のコマンドを使用します：
```bash
ros2 run tf_switch_node tf_switch_node
```

## パラメータ
ノードは以下のパラメータで設定できます：

### フレーム設定
- **`map_frame_id`** (string, デフォルト: "map")
  - マップ座標フレームのフレームID

- **`odom_frame_id`** (string, デフォルト: "odom")
  - オドメトリ座標フレームのフレームID

- **`base_link_frame_id`** (string, デフォルト: "base_link")
  - ロボットベース座標フレームのフレームID

### ノード設定
- **`amcl_node_name`** (string, デフォルト: "amcl")
  - パラメータ制御用のAMCLノード名

- **`publish_tf_param_name`** (string, デフォルト: "tf_broadcast")
  - AMCLの変換ブロードキャストを制御するパラメータ名

### 状態機械パラメータ
- **`covariance_threshold`** (double, デフォルト: 1.5)
  - 定位品質を判定するAMCLポーズ共分散の閾値
  - 値が高いほど定位品質が悪いことを示す
  - この値を超えると状態遷移をトリガーする可能性がある

### 購読トピック
ノードは以下のトピックを購読します：
- **`/fix`** - GNSS状態のためのGPS NavSatFixメッセージ
- **`/amcl_pose`** - 定位品質評価のための共分散付きAMCLポーズ
- **`/odometry/global`** - map->odom変換計算のためのEKFからのグローバルオドメトリ

### 配信トピック
ノードは以下のトピックに配信します：
- **`/initialpose`** - EKFからAMCLへの遷移時のAMCL初期ポーズ
- **`/tf`** - EKFモード時のmapからodomフレームへの変換

### 状態機械ロジック
ノードは3つの状態で動作します：
- **DEAD_RECKON**: GPSもAMCLも信頼できない状態
- **RTK_BAD**: GPSは利用できないがAMCL定位が有効な状態
- **RTK_GOOD**: GPS RTK fixが利用可能で信頼できる状態

パラメータファイルの例 (`tf_switch_params.yaml`):
```yaml
tf_switch_node:
  ros__parameters:
    map_frame_id: "map"
    odom_frame_id: "odom"
    base_link_frame_id: "base_link"
    amcl_node_name: "amcl"
    publish_tf_param_name: "tf_broadcast"
    covariance_threshold: 1.5
```

カスタムパラメータで実行するには：
```bash
ros2 run tf_switch_node tf_switch_node --ros-args --params-file tf_switch_params.yaml
```

## 貢献
貢献を歓迎します！改善提案やプルリクエストをお気軽に送信してください。

1. リポジトリをフォーク
2. フィーチャーブランチを作成 (`git checkout -b feature/AmazingFeature`)
3. 変更をコミット (`git commit -m 'Add some AmazingFeature'`)
4. ブランチにプッシュ (`git push origin feature/AmazingFeature`)
5. プルリクエストを開く

詳細については、[GitHubリポジトリ](https://github.com/iDifferent-studio/tf_switch_node)をご覧ください。

## ライセンス
このプロジェクトはApache-2.0ライセンスの下でライセンスされています。詳細については、LICENSEファイルをご覧ください。

## 依存関係
このパッケージは以下のROS 2パッケージに依存しています：
- `rclcpp` - ROS 2 C++クライアントライブラリ
- `tf2_ros` - TF2 ROSバインディング
- `tf2_geometry_msgs` - TF2ジオメトリメッセージ変換
- `geometry_msgs` - ジオメトリメッセージ定義
- `sensor_msgs` - センサーメッセージ定義
- `nav_msgs` - ナビゲーションメッセージ定義

## バージョン
現在のバージョン：0.1.0
