# 로항 Jetson–QCS 통합

이 저장소는 역할을 다음과 같이 분리합니다.

- **Jetson:** `src/`의 ROS 2 노드, PX4 통신, 비행 명령과 미션 실행
- **노트북:** `qcs/`의 Electron 관제 화면
- **통신:** Jetson HTTP API (`8765/tcp`)

`rohang_qcs/`는 원본 레퍼런스이며 실행에는 프로젝트에 통합된 `qcs/`를
사용합니다. 기존 `src/` 내용은 변경하지 않습니다.

## Jetson

프로젝트를 Jetson에 배포한 후 workspace를 빌드합니다.

```bash
cd /path/to/ws_rh_px15
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

게이트웨이와 기존 제어 노드를 실행합니다.

```bash
./scripts/start_jetson.sh
```

게이트웨이는 기본적으로 `0.0.0.0:8765`에서 수신합니다. 포트를 변경하려면
`JETSON_GCS_PORT`, 수신 주소를 제한하려면 `JETSON_GCS_HOST`를 설정합니다.
기존 `src/` 노드는 현재 운용 절차대로 별도 터미널 또는 launch 파일에서
실행합니다.

## 노트북

최초 한 번 Electron 의존성을 설치합니다.

```bash
cd /path/to/ws_rh_px15/qcs
npm install
```

Jetson의 기본 HM30 주소는 `192.168.144.26`, 게이트웨이 포트는 `8765`로
설정되어 있으므로 별도 환경변수 없이 실행합니다.

```bash
cd /path/to/ws_rh_px15
./scripts/start_qcs.sh
```

Jetson 주소가 달라진 경우에만 실행 전에 `JETSON_GCS_URL`을 지정하면 기본값을
덮어쓸 수 있습니다.

```bash
JETSON_GCS_URL='http://다른-Jetson-IP:8765' ./scripts/start_qcs.sh
```

원격 모드에서는 노트북이 ROS 2, MAVROS, Gazebo 브리지를 실행하지 않습니다.
텔레메트리 조회와 ARM, 비행 모드, VTOL 전환, Plan 검증·업로드·시작 요청은
Jetson 게이트웨이로 전달됩니다.

## 연결 확인

노트북에서 다음 명령의 `ok`가 `true`인지 확인합니다.

```bash
curl http://192.168.144.26:8765/health
```

응답의 `px4Connected`는 Jetson 게이트웨이가 PX4 토픽을 최근 2초 안에
수신했는지를 나타냅니다.

SIYI 영상은 제어 API와 분리해 RTSP 또는 GStreamer H.264/H.265 스트림으로
전송하는 것을 권장합니다.
