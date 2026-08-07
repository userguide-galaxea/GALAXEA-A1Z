# SOP:LubanCat(RK3588)安装 ROS2 Humble

适用设备:LubanCat 主板(Ubuntu 22.04 jammy,arm64),用于跑 `g4spi_node` 和
a1z SDK 的 `transport="g4ros"`。全程约 15-30 分钟(取决于网速)。

## 0. 前置:让板子能上网

板子通过网线接 Mac,走 macOS 的**互联网共享**:

1. Mac 接 Wi-Fi(能上网),网线经 USB 网卡接板子
2. 系统设置 → 通用 → 共享 → 打开"互联网共享",共享到 USB 网卡
3. **如果 Mac 开着全隧道 VPN(流量走 utun):必须先断开**,否则共享的 NAT
   不生效——板子能 ping 通 Mac,但出不了网
4. 板子应自动拿到 `192.168.2.x`;在板子上验证:

```bash
ping -c2 8.8.8.8        # 通 = NAT 正常
```

5. 共享的 DNS 转发不可用,需手动给板子指定 DNS。临时做法(重启丢失):

```bash
sudo resolvectl dns eth1 223.5.5.5 8.8.8.8   # 网卡名以 ip route 为准
```

持久做法(推荐,一次设置永久生效):

```bash
sudo mkdir -p /etc/systemd/resolved.conf.d
printf '[Resolve]\nDNS=223.5.5.5 8.8.8.8\n' | sudo tee /etc/systemd/resolved.conf.d/99-shared-dns.conf
sudo systemctl restart systemd-resolved
```

验证:`python3 -c "import socket; print(socket.gethostbyname('pypi.org'))"`

## 1. 基础工具

```bash
sudo apt-get update
sudo apt-get install -y curl gnupg2 lsb-release ca-certificates locales
```

## 2. ROS2 apt 源(2025 新签名 key + 清华镜像)

**不要**按官方旧文档拿 `ros.key`:`packages.ros.org/ros.key` 和 github raw
上的 key 都是 2025-06-01 过期的旧 key,用它 `apt update` 会报
`EXPKEYSIG F42ED6FBAB17C654`。正确做法是用官方 `ros2-apt-source` deb
(自带轮换后的新 keyring),从清华镜像下载:

```bash
# 1. 下载 ros2-apt-source(含新 key + deb822 源文件)
curl -sSL -o /tmp/ros2-apt-source.deb \
  "https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/pool/main/r/ros-apt-source/ros2-apt-source_1.2.0~jammy_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
#   安装出:/usr/share/keyrings/ros2-archive-keyring.gpg
#          /etc/apt/sources.list.d/ros2.sources

# 2. 把源地址换成清华镜像(jammy = Ubuntu 22.04 的 ROS2 Humble)
sudo sed -i "s|http://packages.ros.org/ros2/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu|g" \
  /etc/apt/sources.list.d/ros2.sources

# 3. ros2.sources 默认带 deb-src,而 tuna 镜像不提供 source 索引(404),
#    把 Types 改成只 deb:
sudo sed -i "s/^Types:.*/Types: deb/" /etc/apt/sources.list.d/ros2.sources

cat /etc/apt/sources.list.d/ros2.sources   # 确认 URIs 指向 tuna、Types 为 deb
```

> 注意:`ros2.sources` 是 deb822 格式(`Types: deb` / `URIs:` /
> `Suites:` 分段),不是老式一行格式;`dpkg -L ros2-apt-source` 列文件时
> 第一行是目录,取文件名要精确匹配 `ros2.sources`。

## 3. 安装 Humble

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base python3-colcon-common-extensions
```

下载量约几百 MB,耐心等。`ros-base` 含 rclpy/rclcpp/常用消息,够跑
`g4spi_node` 和 a1z SDK;如需 RViz 等图形工具再装 `ros-humble-desktop`。

## 4. 验证

```bash
source /opt/ros/humble/setup.bash
python3 -c "import rclpy; print('rclpy OK')"
ros2 --help | head -3
```

建议写进 `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## 5. lemo_main_board 工作空间

仓库:`gitlab.galaxea-ai.com/embeded/a1z_t/lemo_main_board.git`。

> **版本警告(2026-08 实测)**:SDK `lemo` 分支的 g4ros 传输要求
> `MotorData` 逐帧协议(消息 `header / uint16 can_id / uint8 arm_id /
> uint8[] data`,单对 topic `motor_send` / `motor_data`,`arm_id` 1=左
> 2=右)。gitlab 上的 `jsc`/`main` 分支还是旧的 `A1zFrame` 批量协议
> (`MotorCanFrame[6]`、`<side>_motor_send` 双 topic),**不匹配**,编
> 出来 `RosTopicBus` 起不来。目前匹配的实现以嵌入式提供的 tarball 为准;
> 拿到新版本的 `lemo_main_board` 后先核对 `msg/MotorData.msg` 字段再编。

编译(源码、msg 都在仓库里,colcon 已随第 3 步装好):

```bash
cd ~/lemo_main_board
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
python3 -c "from lemo_main_board.msg import MotorData; print('msg OK')"
ros2 pkg list | grep lemo
```

建议把 workspace 的 source 也写进 `~/.bashrc`(非交互 ssh 不读 bashrc,
远程跑命令时要在命令里显式 source):

```bash
echo "source ~/lemo_main_board/install/local_setup.bash" >> ~/.bashrc
```

## 6. 启用 SPI(spidev0.0)

`g4spi_node` 通过 `/dev/spidev0.0` 和 G4 板通信,LubanCat 默认不开:

```bash
# 1. 开 overlay(文件里 overlays= 一行追加,空格分隔)
sudo vi /boot/firmware/ubuntuEnv.txt
#    overlays=... rk3588-lubancat-spi0-m1-overlay

# 2. 重启生效
sudo reboot

# 3. 重启后验证节点存在
ls -l /dev/spidev0.0

# 4. 给普通用户访问权限(免 sudo 跑节点)
echo 'SUBSYSTEM=="spidev", GROUP="spi", MODE="0660"' | \
  sudo tee /etc/udev/rules.d/99-spidev.rules
sudo udevadm control --reload && sudo udevadm trigger
sudo usermod -aG spi $USER   # 重新登录后生效
```

启动透传节点(独占 SPI 轮询,**必须保持运行**,SDK 才有数据):

```bash
nohup ros2 run lemo_main_board g4spi_node > /tmp/g4spi.log 2>&1 &
```

> 注意:节点不随开机自启,板子重启后要重新拉起(或自行做成 systemd
> unit)。日志里刷 "MISO zero / no data received" 属正常——G4 只在有
> 电机流量时才回帧。

## 7. a1z SDK

```bash
cd ~/a1z
pip3 install --user -e . --no-deps --no-build-isolation   # 依赖见下
python3 tools/motor_diag.py --scan                        # 默认 g4ros
```

Python 依赖(`numpy`、`python-can>=4.0`,`pin` 仅动力学需要)。板子能上网
时直接 `pip3 install python-can`;不能上网时在 Mac 上离线搬运:

```bash
# Mac 上:
pip3 download python-can -d /tmp/wheels \
  --platform manylinux2014_aarch64 --python-version 3.10 --only-binary=:all:
rsync -az /tmp/wheels/ cat@192.168.2.2:wheels/
# 板子上:
pip3 install --user --no-index --find-links ~/wheels python-can
```

## 8. 日常使用与排障

跑任何 SDK 脚本前,环境必须 source 好(写进 bashrc 后新开终端即可;
非交互 ssh 要在命令里显式 source):

```bash
source /opt/ros/humble/setup.bash
source ~/lemo_main_board/install/local_setup.bash
```

板子重启后的检查顺序:

```bash
ls /dev/spidev0.0                 # 1. SPI 节点在不在
pgrep -fa g4spi_node              # 2. 透传节点活没活,没在就 nohup 拉起(见第 6 节)
cd ~/a1z && python3 tools/motor_diag.py --scan   # 3. 再跑诊断
```

常见问题:

- **`No module named 'rclpy'` / `requires rclpy and the lemo_main_board
  message package`**:没 source 环境,见上。
- **第一次启动报 "Startup probe did not obtain fresh feedback ... joint*=missing",
  第二次跑就好**:DDS 订阅者发现竞态(冷启动时订阅还没和
  `g4spi_node` 匹配上,启动探测已经发出),重跑一次即可。
- **`motor_data` 上完全没帧**:先确认 `g4spi_node` 活着,再确认电机已
  上电。
- **电机使能/反馈正常但对 MIT 指令完全无响应("假死")**:电机 flash
  里存的控制模式不是 MIT;SDK `enable_all()` 每次启动会统一写回 MIT
  (RAM-only),不用手动处理。
- 测 topic 频率:`ros2 topic hz /motor_data`(反馈)、
  `ros2 topic hz /motor_send`(命令,250Hz 控制时约为 1500Hz)。

## 已知网络限制

- `gitlab.galaxea-ai.com`(192.168.15.10)**只有 VPN 下可达**;断 VPN 开
  共享期间板子访问不到。拉代码走:Mac 上 `git fetch` → `rsync` 到板子
- VPN 重连后共享 NAT 会再次失效,板子断网属预期;需要板子长期在线就把
  它接到办公路由器/交换机上
