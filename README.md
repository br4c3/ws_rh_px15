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

### 5. PX4 DDS와 Jetson 게이트웨이 실행

기본 UDP 연결에서는 다음 명령 하나로 Micro XRCE-DDS Agent와 게이트웨이를
함께 실행합니다.

```bash
./scripts/start_jetson.sh
```

기본값은 UDP 포트 `8888`입니다. PX4가 Jetson 시리얼 포트에 연결되어 있다면:

```bash
PX4_DDS_TRANSPORT=serial \
PX4_DDS_DEVICE=/dev/ttyTHS1 \
PX4_DDS_BAUDRATE=921600 \
./scripts/start_jetson.sh
```

연결 확인:

```bash
curl http://127.0.0.1:8765/health
ros2 topic echo /fmu/out/vehicle_status --once
```

`/health`의 `ok`는 게이트웨이 HTTP 서버 상태이고, 실제 PX4 연결 여부는
`px4Connected`입니다. 다음처럼 나오면 게이트웨이는 실행 중이지만 PX4 DDS
메시지는 아직 들어오지 않는 상태입니다.

```json
{"ok":true,"px4Connected":false}
```

이때 시리얼 연결이라면 `PX4_DDS_TRANSPORT`, 장치 경로, baudrate를 확인하고,
UDP 연결이라면 PX4의 `uxrce_dds_client`가 Jetson 주소의 UDP `8888` 포트를
사용하는지 확인합니다.

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

QGroundControl AppImage를 사용자의 `Downloads` 디렉터리에 둡니다.

```text
~/Downloads/QGroundControl-x86_64.AppImage
```

실행 권한을 부여합니다.

```bash
chmod +x ~/Downloads/QGroundControl-x86_64.AppImage
```

QCS는 `Downloads`와 `~/apps`의 일반적인 AppImage 이름을 자동으로 찾습니다.
다른 위치에 설치했다면 QCS 실행 시 `QGC_PATH`로 경로를 지정할 수 있습니다.

Electron은 현재 데스크톱 세션의 Wayland/X11 환경을 자동으로 선택합니다.
QGroundControl은 호환성을 위해 XWayland(`xcb`)와 OpenGL 렌더러를 사용합니다.

QCS 실행:

```bash
./scripts/start_qcs.sh
```

스크립트는 `node`와 `npm`이 PATH에 없으면 `~/.nvm/nvm.sh`를 자동으로
불러옵니다. Jetson 주소가 기본값 `192.168.144.26`과 다르면 다음처럼
지정합니다.

```bash
JETSON_GCS_URL=http://JETSON_IP:8765 ./scripts/start_qcs.sh
```

시작 시 표시되는 상태는 다음과 같습니다.

- `PX4 DDS connected`: Jetson과 PX4 데이터가 모두 정상입니다.
- `gateway is reachable, but PX4 DDS is not connected`: Jetson HTTP 서버는
  정상이지만 PX4 DDS 연결을 확인해야 합니다.
- `Jetson gateway is not reachable`: Jetson IP, 포트, 방화벽 또는
  `start_jetson.sh` 실행 상태를 확인해야 합니다.

Wayland에서는 화면 공유 창의 반복 표시를 막기 위해 QGC 화면 미러링이 기본적으로
꺼집니다. 미러링이 필요하면 `QGC_CAPTURE=1 ./scripts/start_qcs.sh`로 실행합니다.

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
