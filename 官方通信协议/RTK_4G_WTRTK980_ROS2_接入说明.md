# RTK_4G 评估板 + WTRTK-980 接入 ROS2 说明

本文用于说明 RTK_4G 评估板搭配 WTRTK-980 开发板的使用逻辑，并给 ROS2 GPS/RTK 导航节点开发提供背景。

## 1. 硬件关系

RTK_4G 评估板不是定位模块本体，它主要负责联网、Ntrip/CORS 差分获取、数据转发和配置。

WTRTK-980 才是真正的 GNSS/RTK 定位模块，负责接收卫星信号、接收 RTCM 差分数据，并输出最终定位结果。

整体链路如下：

```text
Ntrip/CORS 差分服务器
        |
      4G 网络 / SIM 卡
        |
RTK_4G 评估板
        |
   RTCM 差分数据
        |
WTRTK-980 RTK 模块
        |
NMEA 定位数据: GGA / RMC / VTG / GST ...
        |
ROS2 机器人
```

因此，ROS2 侧通常不需要自己实现 Ntrip 客户端。当前硬件架构下，RTK_4G 评估板负责连接 Ntrip/CORS，ROS2 只读取 WTRTK-980 解算后的 NMEA 定位数据。

## 2. Type-C 接口选择

如果目标是：

```text
RTK + 4G 差分 + ROS2 小车定位
```

ROS2 主机优先连接：

```text
RTK_4G 评估板 Type-C
```

WTRTK-980 自己的 Type-C 更适合单独调试 RTK 模块，例如使用 UPrecise 查看效果、发送 UM980 模块命令等。

RTK_4G 评估板 Type-C 在 PC 上可能枚举出多个 COM 口。厂家说明中提到：评估板虚拟串口默认打开第一个串口号通信即可。但实际工程中必须逐个测试，找到实际可用端口。

本次实测中，Windows 枚举出 4 个 COM 口，实际使用的是第二个端口。该端口同时具备两个特征：

```text
1. 发送 AT+CFG=? 会返回 RTK_4G 评估板配置。
2. 持续输出 WTRTK-980 的 NMEA 定位数据。
```

因此，在本次硬件组合中，第二个端口可作为：

```text
RTK_4G 配置口，同时也是 ROS2 NMEA 数据读取口
```

不要只根据“第几个 COM 口”写死程序。移入 ROS2 小车后，应按端口内容判断：能持续输出 `$GNGGA`、`$GNRMC`，并且能通过 `AT+CFG=?` 返回配置的端口，就是当前硬件的主数据/配置口。

常见 NMEA 输出类似：

```text
$GNGGA,...
$GNRMC,...
$GNVTG,...
```

ROS2 应读取这个持续输出 NMEA 的串口。

本次实测可用端口输出示例：

```text
$GNRMC,061801.00,A,3043.22676844,N,11118.37324088,E,0.003,139.4,230626,4.0,W,F,C*56
$GNGGA,061801.00,3043.22676844,N,11118.37324088,E,5,21,0.9,92.8063,M,-22.5265,M,1.0,1690*4E
```

## 3. 串口参数

默认串口参数：

```text
baudrate: 115200
data bits: 8
parity: none
stop bits: 1
```

Linux 下可能枚举为：

```text
/dev/ttyUSB0
/dev/ttyUSB1
/dev/ttyUSB2
/dev/ttyACM0
```

可以逐个检查：

```bash
screen /dev/ttyUSB0 115200
```

看到 `$GNGGA`、`$GNRMC` 等连续输出的端口，就是 ROS2 GPS 节点应使用的端口。

在 ROS2 小车上建议这样确认端口：

```bash
ls /dev/ttyUSB*
dmesg | tail -n 50
```

逐个打开：

```bash
screen /dev/ttyUSB0 115200
screen /dev/ttyUSB1 115200
screen /dev/ttyUSB2 115200
screen /dev/ttyUSB3 115200
```

如果某个端口可以看到 `$GNGGA`、`$GNRMC`，该端口就是 ROS2 节点读取口。

如果需要确认它也是配置口，可以在串口工具中发送：

```text
AT+CFG=?
```

正确配置口会返回：

```text
+NTRIPEN=1
+ADDR=<server_address>,<port>
+CORS=<user>,<password>,<mount_point>
+CCID=<sim_iccid>
+IMEI=<module_imei>
+VERSION=<firmware_version>
OK
```

为了避免重启后 `/dev/ttyUSBx` 编号变化，正式上车建议写 udev 规则，把该端口固定成稳定名称，例如：

```text
/dev/rtk_4g
```

udev 规则需要根据实际设备的 `idVendor`、`idProduct`、`serial` 或 `bInterfaceNumber` 编写。先查看：

```bash
udevadm info -a -n /dev/ttyUSB1
```

其中 `/dev/ttyUSB1` 替换为实测 NMEA/配置端口。

## 4. Ntrip/CORS 配置

CORS/Ntrip 账号不是在 ROS2 节点里输入的，而是预先配置到 RTK_4G 评估板中。

上位机中的 Ntrip 配置项：

```text
服务器地址：Ntrip 服务器 IP 或域名
端口：Ntrip 端口
账号：CORS / Ntrip 账号
密码：CORS / Ntrip 密码
挂载点：mount point
```

账号、密码、挂载点来自千寻、移动 CORS、省网 CORS、商家测试账号或自建基站服务。

注意：

```text
这不是 SIM 卡账号。
这不是 Wi-Fi 账号。
这不是 ROS2 账号。
```

也可以通过串口 AT 指令配置：

```text
AT+CORS=<user>,<password>,<mount_point>
AT+ADDR=<server_address>,<port>
AT+CFG=?
```

发送 AT 指令时需要加回车换行。收到 `OK` 表示配置成功。

注意：`AT+CORS` 中间必须使用英文逗号，挂载点前后不能有空格。Ntrip 服务器通常会把尾部空格当成 mount point 字符的一部分。

错误示例：

```text
AT+CORS=user,password,RTCM32_GRECJ2  
```

上面 `RTCM32_GRECJ2` 后面如果有两个空格，可能导致 Ntrip 登录失败或无法获取差分。

正确示例：

```text
AT+CORS=user,password,RTCM32_GRECJ2
```

本次实测中，挂载点尾部存在空格时，GGA quality 一直停在 `1`。去掉尾部空格并重新配置后，GGA quality 成功变为 `5`，说明差分已经进入 WTRTK-980。

如果配置错误，可以恢复默认参数：

```text
AT+RESTORE
```

## 5. 指示灯状态

`WORK` 灯状态：

```text
未入网：亮 1 秒，灭 1 秒
已入网：一秒闪一次
连接 Ntrip 成功：一秒闪两次
```

`PPS` 灯连接 RTK 模块 PPS / 定位状态，但不能单独作为 RTK Fixed 的判断依据。

RTK 是否固定，建议看 NMEA `$GNGGA` 的 fix quality 字段。

常见 GGA quality：

```text
0 = 无定位
1 = 单点定位
2 = DGPS
4 = RTK Fixed
5 = RTK Float
```

ROS2 节点应解析 GGA，并把 RTK Fixed / Float 状态暴露出来，方便导航系统判断定位质量。

本次实测状态变化：

```text
室内或天线环境差：
GGA quality = 0, satellites = 00

室外开阔但差分未生效：
GGA quality = 1, satellites = 20+

去掉 CORS 挂载点尾部空格后：
GGA quality = 5, differential age = 1.0, base station ID = 1690
```

示例：

```text
$GNGGA,061801.00,3043.22676844,N,11118.37324088,E,5,21,0.9,92.8063,M,-22.5265,M,1.0,1690*4E
```

字段含义：

```text
E 后面的 5 = RTK Float
21 = 参与定位卫星数
0.9 = HDOP
1.0 = 差分龄期，说明 RTCM 差分正在进入模块
1690 = 差分基站 ID
```

如果从 `5` 变成 `4`，表示 RTK Fixed。`5` 说明差分已经成功，但载波相位固定模糊度尚未固定。此时需要更开阔环境、稳定天线、等待收敛。

## 6. WTRTK-980 模块工作模式

UM980 / WTRTK-980 默认可作为 Rover 使用。常用命令：

```text
MODE ROVER
SAVECONFIG
```

模块能自适应识别 RTCM 差分输入。

RTK_4G 评估板检测到 2 秒内无 GGA 输出时，会尝试自动配置 RTK 模块 1 Hz GGA 回传，但不保存到 Flash。

如果需要手动配置 NMEA 输出，可参考 UM980 命令：

```text
GPGGA COMx 1
GPRMC COMx 1
GPVTG COMx 1
SAVECONFIG
```

实际输出可能是 `$GPGGA`，也可能是 `$GNGGA`。ROS2 节点应兼容 `GP` 和 `GN` talker ID。

## 7. ROS2 节点建议

第一版建议先验证串口和 NMEA 数据通路，可以使用现成包：

```bash
sudo apt install ros-$ROS_DISTRO-nmea-navsat-driver
ros2 run nmea_navsat_driver nmea_serial_driver \
  --ros-args \
  -p port:=/dev/ttyUSB1 \
  -p baud:=115200 \
  -p frame_id:=gps_link
```

上面 `/dev/ttyUSB1` 只是本次实测中“第二个端口”的 Linux 可能形式。实际移入小车时必须以 `screen` 或 `udevadm` 确认，正式配置推荐使用 udev 固定名，例如：

```bash
ros2 run nmea_navsat_driver nmea_serial_driver \
  --ros-args \
  -p port:=/dev/rtk_4g \
  -p baud:=115200 \
  -p frame_id:=gps_link
```

检查输出：

```bash
ros2 topic echo /fix
ros2 topic hz /fix
```

如果 `/fix` 有经纬高数据，说明 RTK 到 ROS2 的数据链路已经通了。

还需要额外检查 RTK 状态。仅有 `/fix` 不代表厘米级 RTK。必须确认 GGA quality：

```text
1 = 单点，米级
5 = RTK Float，差分已进入，未固定
4 = RTK Fixed，厘米级
```

如果使用 `nmea_navsat_driver` 后无法直接看到 GGA quality，建议同时保留原始 NMEA 话题，或自定义节点发布 `gga_quality`、`rtk_state`、`differential_age`、`base_station_id`。

自定义 ROS2 节点最小功能：

```text
读取串口 NMEA
解析 GGA / RMC / VTG
发布 sensor_msgs/NavSatFix
可选发布 geometry_msgs/TwistStamped 或 nav_msgs/Odometry
发布 RTK 状态诊断信息
```

建议自定义节点从 GGA 中额外解析：

```text
gga_quality
satellite_count
hdop
altitude_msl
geoid_separation
differential_age
base_station_id
```

本次实测中，`differential_age=1.0` 和 `base_station_id=1690` 出现时，说明差分链路已经实际生效。

推荐话题：

```text
/fix                  sensor_msgs/NavSatFix
/gps/vel              geometry_msgs/TwistStamped，可选
/gps/status           diagnostic_msgs/DiagnosticArray 或自定义消息
/nmea_sentence        nmea_msgs/Sentence，可选
```

`NavSatFix.status.status` 可参考如下映射：

```text
GGA quality 0 -> STATUS_NO_FIX
GGA quality 1 -> STATUS_FIX
GGA quality 2 -> STATUS_SBAS_FIX
GGA quality 4 -> STATUS_GBAS_FIX
GGA quality 5 -> STATUS_GBAS_FIX
```

由于 ROS 标准 `NavSatStatus` 无法区分 RTK Fixed 和 RTK Float，建议额外发布原始 `gga_quality` 或自定义 RTK 状态。

## 8. 导航融合建议

RTK 输出的是经纬度，不是 ROS 的 `map` 或 `odom` 坐标。

导航融合通常需要：

```text
RTK GPS: sensor_msgs/NavSatFix
IMU: sensor_msgs/Imu
轮速里程计: nav_msgs/Odometry
robot_localization:
  ekf_node
  navsat_transform_node
```

典型链路：

```text
/fix + /imu/data + /wheel/odom
        |
robot_localization
        |
/odometry/filtered
/map -> /odom -> /base_link
```

需要配置 GPS 天线相对车体的 TF，例如：

```text
base_link -> gps_link
```

如果 GPS 天线不在车体中心，应配置天线相对 `base_link` 的偏移，否则转弯和融合时会产生系统误差。

## 9. 给 ROS2 节点开发的重点提醒

当前硬件设计中，RTK_4G 评估板负责 Ntrip/CORS，不建议第一版 ROS2 节点再实现 Ntrip 客户端。

节点需要重点处理：

```text
多 COM / 多 tty 情况下选择正确串口
NMEA 校验和
$GNGGA / $GPGGA 兼容
$GNRMC / $GPRMC 兼容
RTK Fixed / Float 状态输出
串口断线重连
无 GGA 超时报警
经纬高协方差估计
```

建议开发顺序：

```text
1. 用 nmea_navsat_driver 验证串口数据通路。
2. 确认能稳定发布 /fix。
3. 自定义节点增强 RTK 状态、诊断、速度、协方差。
4. 接入 robot_localization 做融合。
5. 最后再根据导航需求调整话题名、frame_id 和 TF。
```

## 10. 本次硬件实测结论

本次测试环境和现象汇总：

```text
硬件：RTK_4G 评估板 + WTRTK-980
连接：PC / ROS2 主机连接 RTK_4G 评估板 Type-C
Windows 枚举：4 个 COM 口
实测可用：第二个端口
波特率：115200
```

第二个端口表现：

```text
发送 AT+CFG=? 能返回 RTK_4G 配置。
持续输出 $GNGGA / $GNRMC / $GPGSV / $GBGSV 等 NMEA 数据。
可用于配置 CORS/Ntrip。
可用于 ROS2 读取定位数据。
```

配置成功但差分未生效时：

```text
$GNGGA,...,E,1,27,0.6,...
```

含义：

```text
E = 东经，不是状态
1 = 单点定位
27 = 卫星数，说明 GNSS 环境很好
0.6 = HDOP，说明普通定位质量较好
```

差分生效后：

```text
$GNGGA,...,E,5,21,0.9,...,1.0,1690*...
```

含义：

```text
5 = RTK Float
1.0 = 差分龄期
1690 = 差分基站 ID
```

后续上车目标：

```text
1. 用第二个端口对应的 Linux tty 作为 ROS2 串口输入。
2. 用 udev 固定该端口名称。
3. ROS2 节点读取 NMEA 并发布 /fix。
4. 自定义或扩展节点发布 RTK Float / Fixed 状态。
5. 与 IMU、轮速通过 robot_localization 融合。
```

## 11. 上车检查清单

上车前检查：

```text
RTK GNSS 天线接到 WTRTK-980 的 GNSS 天线口。
4G 天线 / 板载 4G 天线信号正常。
SIM 卡有流量，未停机。
CORS 账号、密码、挂载点正确，挂载点无尾部空格。
RTK_4G 板拨码/开关处于可控制 4G 评估板和 RTK 模块的位置。
```

上车后 Linux 检查：

```bash
ls /dev/ttyUSB*
screen /dev/ttyUSB1 115200
```

看到如下内容表示串口正确：

```text
$GNGGA,...
$GNRMC,...
```

看到如下内容表示差分已经进入：

```text
$GNGGA,...,E,5,...,<differential_age>,<base_station_id>*...
```

看到如下内容表示 RTK Fixed：

```text
$GNGGA,...,E,4,...
```

ROS2 启动前建议固定端口：

```text
/dev/rtk_4g
```

ROS2 配置中使用：

```text
port: /dev/rtk_4g
baud: 115200
frame_id: gps_link
```





