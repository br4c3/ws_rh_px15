# 설치 방법

## Jetson

### 1. Ubuntu 버전 확인

```bash
lsb_release -sc
```

ROS 2 Humble 바이너리 패키지를 설치하려면 `jammy`(Ubuntu 22.04)가 나와야
합니다. `focal`(Ubuntu 20.04)에서는 아래 apt 설치 방법을 사용할 수 없습니다.

### 2. ROS 2 apt 저장소 등록

```bash
sudo apt update
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe

sudo curl -sSL \
  https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt update
```

### 3. ROS 2와 MAVROS 설치

```bash
sudo apt install -y \
  ros-humble-ros-base \
  ros-humble-mavros \
  ros-humble-mavros-msgs \
  ros-humble-mavros-extras \
  python3-colcon-common-extensions \
  python3-rosdep
```

설치 확인:

```bash
source /opt/ros/humble/setup.bash
python3 -c "import rclpy, mavros_msgs; print('ROS 2 + MAVROS OK')"
```

### 4. workspace 의존성 설치 및 빌드

```bash
cd /path/to/ws_rh_px15

sudo rosdep init 2>/dev/null || true
rosdep update

source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

빌드 결과 확인:

```bash
source install/setup.bash
ros2 pkg list | grep guidance
```

## 노트북

### 1. Node.js와 npm 설치

```bash
sudo apt update
sudo apt install -y nodejs npm
```

### 2. Electron QCS 의존성 설치

```bash
cd /path/to/ws_rh_px15/qcs
npm install
```

### 3. QGroundControl 설치

QGroundControl AppImage를 다음 위치에 둡니다.

```text
/home/br4c3/apps/QGroundControl.AppImage
```

실행 권한을 부여합니다.

```bash
chmod +x /home/br4c3/apps/QGroundControl.AppImage
```

다른 위치에 설치했다면 QCS 실행 시 `QGC_PATH`로 경로를 지정할 수 있습니다.

---

## 이상한 빌드 버그

```bash
cd ~/ws_rh_px15

source /opt/ros/humble/setup.bash

rm -rf build/mavros_msgs install/mavros_msgs

colcon build \
  --packages-select mavros_msgs \
  --symlink-install
```