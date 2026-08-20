# Indoor Mobile Robot - Teensy Controller

Teensy 4.1で差動二輪ロボットのモーター制御、エンコーダ計測、オドメトリ計算を行い、micro-ROSを介してRaspberry Pi上のROS 2と通信するファームウェアです。

## 主な機能

- `/cmd_vel`を購読し、並進速度と角速度から左右車輪の目標角速度を計算
- A/B相エンコーダから左右車輪の回転速度を計測
- フィードフォワード＋PID制御で左右モーターへPWMを出力
- 車輪速度から差動二輪の位置・姿勢を推定
- `/odom`へ位置、姿勢、並進速度、角速度を20 Hzで配信
- micro-ROS Agentの停止と復帰を検知し、USBを挿し直さず自動再接続
- Agent切断時および`/cmd_vel`タイムアウト時に目標速度をゼロ化

## システム構成

```text
ROS 2ノード
  │  /cmd_vel, /odom
Raspberry Pi
  │  micro_ros_agent（serial transport）
  │  /dev/microros
USB Serial
  │
Teensy 4.1
  ├─ micro-ROS client
  ├─ 差動二輪オドメトリ
  ├─ 左右モーター制御
  └─ 左右エンコーダ計測
```

micro-ROSはROS 2 Jazzy向けに構成されています。

## ROS 2インターフェース

| 種別 | 名前 | 型 | 用途 |
| --- | --- | --- | --- |
| Subscriber | `/cmd_vel` | `geometry_msgs/msg/Twist` | 車体の目標並進速度と目標角速度 |
| Publisher | `/odom` | `nav_msgs/msg/Odometry` | 推定位置、姿勢、車体速度 |

### `/cmd_vel`から車輪速度への変換

`linear.x`を車体並進速度 `v`、`angular.z`をヨー角速度 `w` として、左右車輪の目標角速度を次式で求めます。

```text
right = (v + w * wheel_base / 2) / wheel_radius
left  = (v - w * wheel_base / 2) / wheel_radius
```

計算結果は`MAX_WHEEL_RAD_S`の範囲に制限され、各モーターの速度制御へ渡されます。

## 制御処理

`loop()`では、micro-ROSの接続管理とは独立してエンコーダ計測とモーター制御を継続します。

1. エンコーダのカウント値から左右車輪の角速度を更新する
2. `/cmd_vel`から求めた左右車輪の目標角速度を取得する
3. フィードフォワードとPIDからPWM指令を計算する
4. Hブリッジの2入力へ正転・逆転に対応したPWMを出力する
5. 左右車輪速度を積分して `x`、`y`、`theta` を更新する
6. 接続中は計算結果を `/odom` へ配信する

モーター出力が飽和している方向には積分値を増加させない簡易アンチワインドアップを実装しています。目標速度がほぼゼロの場合はPWMをゼロにし、PID積分値もリセットします。

## micro-ROS Agent自動再接続

Teensyは次の状態機械でAgentとの接続を管理します。

```text
WAITING_AGENT
  │ rmw_uros_ping_agent()成功
  ▼
AGENT_AVAILABLE
  │ node / publisher / subscriber / executor生成成功
  ▼
AGENT_CONNECTED
  │ executor実行、500 msごとにAgentをping
  │ ping失敗
  ▼
AGENT_DISCONNECTED
  │ 目標速度をゼロ化
  │ 既存entityを破棄
  └──────────────► WAITING_AGENT
```

### 接続時

以下を順番に実行します。

1. `rclc_support_init()`でmicro-ROSコンテキストを初期化
2. `rmw_uros_sync_session()`でAgentと時刻同期
3. `teensy_base_controller`ノードを生成
4. `/odom` publisherを生成
5. `/cmd_vel` subscriberを生成
6. executorへsubscriberを登録

### 切断時

Agentが停止してpingに失敗すると、まず車輪目標速度をゼロにします。その後、executor、subscriber、publisher、node、supportを破棄し、Agent待機状態へ戻ります。

Agentが存在しない状態でTeensyを起動しても停止せず、500 msごとにAgentを探します。Agentが復帰するとentityを新しく生成するため、TeensyのリセットやUSBの挿し直しは不要です。

entity破棄時のセッションタイムアウトは0に設定し、Agent停止中の破棄処理が長時間ブロックすることを防いでいます。初期化途中で失敗した場合は、実際に生成できたentityだけを破棄します。

## 安全動作

- Agent待機中と切断検知後は左右車輪の目標速度をゼロにします。
- Agent未接続時は `/odom` をpublishしません。
- `/cmd_vel`を一定時間受信しなければ目標速度をゼロにします。
- micro-ROS通信に使用する`Serial`へのデバッグ文字列出力は無効にしています。

`Serial`にはXRCE-DDSのバイナリデータが流れます。同じ`Serial`へ`Serial.print()`などで文字列を出力すると通信を破損する可能性があります。デバッグには別UART、別USBシリアル、またはLEDを使用してください。

## 主な設定

設定値は [`include/config.hpp`](include/config.hpp) にあります。

| 設定 | 現在値 | 内容 |
| --- | ---: | --- |
| `CONTROL_PERIOD` | 10 ms | モーター制御周期 |
| `ODOM_PUBLISH_PERIOD` | 50 ms | `/odom`配信周期（20 Hz） |
| `CMD_VEL_TIMEOUT_MS` | 1,000,000 ms | `/cmd_vel`無受信時の停止判定 |
| `WHEEL_RADIUS` | 0.03357 m | 車輪半径 |
| `WHEEL_BASE` | 0.29398 m | 左右車輪間距離 |
| `MAX_WHEEL_RAD_S` | 5.0 rad/s | 車輪目標角速度の上限 |
| `PWM_BIT` | 12 bit | PWM分解能 |
| `PWM_FREQ_HZ` | 1,000 Hz | PWM周波数 |
| `PING_INTERVAL_MS` | 500 ms | Agent死活監視周期 |

`CMD_VEL_TIMEOUT_MS`の現在値は1,000秒です。運用上すぐ停止させたい場合は、例えば1秒なら`1000`へ変更してください。

ピン割り当て、エンコーダ極性、モーター出力極性、PIDゲイン、フィードフォワード係数も同じファイルで変更できます。

## ソース構成

| ファイル | 内容 |
| --- | --- |
| `src/main.cpp` | ハードウェア初期化とメイン制御ループ |
| `src/telemetry.cpp` | micro-ROS entity、トピック処理、Agent再接続状態機械 |
| `src/encoder.cpp` | エンコーダカウントと車輪速度計測 |
| `src/motor.cpp` | PID、フィードフォワード、PWM出力 |
| `src/odom.cpp` | 差動二輪オドメトリ計算 |
| `include/config.hpp` | ピン、車体寸法、周期、制御ゲインなどの設定 |
| `platformio.ini` | Teensy 4.1およびmicro-ROS Jazzyのビルド設定 |

## ビルドと書き込み

プロジェクトディレクトリで実行します。

```bash
pio run -e teensy41
pio run -e teensy41 -t upload
```

生成されるファームウェアは次の場所にあります。

```text
.pio/build/teensy41/firmware.hex
```

## Agent起動例

Raspberry Pi側でデバイスが`/dev/microros`として利用できる場合の例です。

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/microros -v6
```

常時運用ではmicro-ROS Agentをsystemdで管理し、少なくとも次の再起動設定を使用します。

```ini
Restart=always
RestartSec=1
```

## 自動再接続の確認

1. Teensyを書き込み後、micro-ROS Agentを起動する
2. `ros2 topic list`で`/cmd_vel`と`/odom`を確認する
3. `ros2 topic echo /odom`でデータ受信を確認する
4. USBを抜かずにAgentだけを停止する
5. Agentを再起動する
6. `/odom`の配信が自動的に再開することを確認する

Agent再起動後にログが再びentity生成まで進めば、再接続は成功です。
