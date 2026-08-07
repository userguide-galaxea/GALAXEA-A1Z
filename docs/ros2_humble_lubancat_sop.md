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

> **版本警告(2026-08 实测)**:gitlab 上的 `jsc`/`main`/`LGN` 分支都还是
> `A1zFrame` 批量协议(`MotorCanFrame[6]`),**没有** a1z `lemo` 分支
> `ros_topic_bus.py` 所需的逐帧 `CanFrame`(`header/arm_id/id/不定长data`)
> 消息和 `<side>_motor_send/data` topic。跑 SDK 的 g4ros 传输前,先确认
> 嵌入式已提供匹配版本的 `lemo_main_board`,否则 `RosTopicBus` 起不来。

编译(源码、msg 都在仓库里,colcon 已随第 3 步装好):

```bash
cd ~/lemo_main_board
rm -rf build install log
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
python3 -c "from lemo_main_board.msg import CanFrame; print('msg OK')"
ros2 pkg list | grep lemo
```

启动透传节点(独占 SPI 轮询,需保持运行):

```bash
ros2 run lemo_main_board g4spi_node
```

## 6. a1z SDK

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

## 已知网络限制

- `gitlab.galaxea-ai.com`(192.168.15.10)**只有 VPN 下可达**;断 VPN 开
  共享期间板子访问不到。拉代码走:Mac 上 `git fetch` → `rsync` 到板子
- VPN 重连后共享 NAT 会再次失效,板子断网属预期;需要板子长期在线就把
  它接到办公路由器/交换机上
