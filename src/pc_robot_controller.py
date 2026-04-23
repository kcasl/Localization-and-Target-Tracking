import math
import socket
import time

import cv2
import numpy as np

# TODO: 환경에 맞게 수정
ESP_IP = "10.31.216.22"
ESP_PORT = 4210
# USB 웹캠을 우선 사용 (Windows에서 보통 1 이상)
CAMERA_INDEX = 2
FRAME_WIDTH = 1920
FRAME_HEIGHT = 1080
MIRROR_VIEW = False
# marker.png 기준: OpenCV ArUco 사전 DICT_ARUCO_MIP_36H12, ID 129
# 다른 마커를 쓰면 src/identify_marker.py 로 확인 후 아래 두 값만 바꾸면 됨
ARUCO_DICT = cv2.aruco.DICT_ARUCO_MIP_36H12
MARKER_ID = 129

# 모터 출력 약 1.5배 (하드웨어 상한 255)
MAX_PWM = 255
BASE_PWM = 165
KP_ANGLE = 120.0
KP_DISTANCE = 0.9
STOP_DIST_PX = 4
LOOP_DELAY_SEC = 0.02
TURN_ONLY_THRESHOLD_DEG = 20.0  # 이 각도보다 크면 한쪽 정지 회전
TURN_PWM = 180  # 한쪽 정지 회전 시 구동 PWM
# 빨간 영역이 작게 보이면 줄이고, 잡음이 많으면 늘리기
MIN_RED_AREA_PX = 400
# 거리 최소화 집중 제어 파라미터
ALIGN_DEG = 7.0  # 이 각도 이내가 되면 직진
DRIVE_MIN_PWM = 158
DIST_RISING_TOL_PX = 2.0  # 이 값 이상 거리 증가 시 "멀어짐"으로 판단
WRONG_WAY_LIMIT = 6  # 연속 멀어짐 횟수 초과 시 전진 부호 반전
# 목표까지 조금 더 밀어붙이기 위한 가상 거리 보정(클수록 더 적극적으로 접근)hnn
APPROACH_EXTRA_PX = 10.0
# 차체 기준 재정의:
# 기존 "전진(+)" 명령이 실제 후진이라면 -1, 반대면 1
FORWARD_SIGN = 1
# 인식표 부착 방향 보정(로봇 전방과 마커 기준이 다르면 각도 오차 발생)
# 빨간 목표에 접근하도록 기본 방향(0도)으로 복귀
MARKER_HEADING_OFFSET_DEG = 0.0
# 피벗 회전 시 각 모드별 좌/우 바퀴 명령 방향(+1 정방향, -1 역방향)
PIVOT_LEFT_LEFT_SIGN = 0
PIVOT_LEFT_RIGHT_SIGN = 1
PIVOT_RIGHT_LEFT_SIGN = 1
PIVOT_RIGHT_RIGHT_SIGN = 0

# 노트북 ↔ ESP8266 UDP 연결 확인 (펌웨어: "PING" 수신 시 "PONG" 회신)
LINK_PING = b"PING"
LINK_PONG_PREFIX = b"PONG"
LINK_START_RETRIES = 3
LINK_START_TIMEOUT_SEC = 2.0
LINK_POLL_INTERVAL_SEC = 5.0
LINK_POLL_RECV_TIMEOUT_SEC = 0.25
CAMERA_BACKEND = cv2.CAP_DSHOW  # Windows USB 카메라 연결 안정화


def clamp(value, low, high):
    return max(low, min(high, value))


def to_drive_axis(value, drive_sign):
    """제어계(+전진)를 실제 차체 모터 축 명령으로 변환."""
    return int(drive_sign * value)


def open_camera():
    """설정 인덱스를 우선, 실패 시 0 인덱스로 재시도."""
    cap = cv2.VideoCapture(CAMERA_INDEX, CAMERA_BACKEND)
    if cap.isOpened():
        return cap, CAMERA_INDEX

    # USB 인덱스가 환경마다 다를 수 있어 내장캠으로 fallback
    cap = cv2.VideoCapture(0, CAMERA_BACKEND)
    if cap.isOpened():
        return cap, 0
    return cap, None


def send_motor(sock, left, right):
    msg = f"{int(left)},{int(right)}".encode("utf-8")
    sock.sendto(msg, (ESP_IP, ESP_PORT))


def verify_esp_link_at_start(sock):
    """시작 시 하드웨어(ESP8266)와 UDP 왕복 여부 확인."""
    print(
        f"[연결 확인] ESP8266으로 PING 전송 중… 대상 {ESP_IP}:{ESP_PORT} "
        f"(최대 {LINK_START_RETRIES}회, 응답 대기 {LINK_START_TIMEOUT_SEC}s)"
    )
    for attempt in range(1, LINK_START_RETRIES + 1):
        try:
            sock.sendto(LINK_PING, (ESP_IP, ESP_PORT))
            sock.settimeout(LINK_START_TIMEOUT_SEC)
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            print(f"[연결 확인] 시도 {attempt}/{LINK_START_RETRIES}: PONG 없음 (타임아웃)")
            continue
        finally:
            sock.settimeout(None)

        if data.startswith(LINK_PONG_PREFIX) and addr[0] == ESP_IP:
            print(
                f"[연결 확인] 성공: 노트북 ↔ ESP8266 ({ESP_IP}) "
                f"응답 {addr[0]}:{addr[1]} 수신"
            )
            return True
        print(
            f"[연결 확인] 시도 {attempt}/{LINK_START_RETRIES}: "
            f"예상과 다른 응답 (from {addr[0]}:{addr[1]} payload={data!r})"
        )

    print(
        "[연결 확인] 실패: ESP8266에서 PONG이 오지 않습니다. "
        "ESP_IP, 동일 Wi-Fi, Windows 방화벽(UDP), 펌웨어 업로드·시리얼 로그를 확인하세요."
    )
    return False


def poll_esp_link(sock):
    """주기적 연결 확인. 성공 시 True."""
    try:
        sock.sendto(LINK_PING, (ESP_IP, ESP_PORT))
        sock.settimeout(LINK_POLL_RECV_TIMEOUT_SEC)
        data, addr = sock.recvfrom(256)
    except socket.timeout:
        return False
    finally:
        sock.settimeout(None)

    return bool(data.startswith(LINK_PONG_PREFIX) and addr[0] == ESP_IP)


def detect_red_candidates(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # red range
    lower1 = np.array([0, 100, 80])
    upper1 = np.array([10, 255, 255])
    lower2 = np.array([160, 100, 80])
    upper2 = np.array([179, 255, 255])
    mask1 = cv2.inRange(hsv, lower1, upper1)
    mask2 = cv2.inRange(hsv, lower2, upper2)
    red_mask = cv2.bitwise_or(mask1, mask2)

    # yellow range
    lower_yellow = np.array([18, 90, 90])
    upper_yellow = np.array([38, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # red + yellow 통합
    mask = cv2.bitwise_or(red_mask, yellow_mask)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < MIN_RED_AREA_PX:
            continue

        m = cv2.moments(contour)
        if m["m00"] == 0:
            continue

        cx = float(m["m10"] / m["m00"])
        cy = float(m["m01"] / m["m00"])
        candidates.append((cx, cy, area))

    return candidates


def select_front_target(candidates, robot_x, robot_y, robot_heading):
    """빨간 후보 중 로봇 진행방향(전방)에 있는 목표만 선택."""
    hx = math.cos(robot_heading)
    hy = math.sin(robot_heading)

    best = None
    best_forward_proj = -1e9
    best_area = -1.0
    for cx, cy, area in candidates:
        vx = cx - robot_x
        vy = cy - robot_y
        forward_proj = vx * hx + vy * hy
        if forward_proj <= 0:
            continue
        # 전방 투영이 큰 후보를 우선, 동률이면 큰 면적 우선
        if forward_proj > best_forward_proj or (
            abs(forward_proj - best_forward_proj) < 1e-6 and area > best_area
        ):
            best_forward_proj = forward_proj
            best_area = area
            best = (int(cx), int(cy))

    return best


def detect_robot_pose(frame, detector):
    corners, ids, _ = detector.detectMarkers(frame)
    if ids is None:
        return None

    ids = ids.flatten()
    for idx, marker_id in enumerate(ids):
        if marker_id == MARKER_ID:
            pts = corners[idx][0]
            center = np.mean(pts, axis=0)

            # ArUco corner 순서 (좌상, 우상, 우하, 좌하)
            p0 = pts[0]
            p1 = pts[1]
            heading = p1 - p0
            theta = math.atan2(heading[1], heading[0])

            return (center[0], center[1], theta, pts.astype(int))
    return None


def wrap_angle(rad):
    while rad > math.pi:
        rad -= 2 * math.pi
    while rad < -math.pi:
        rad += 2 * math.pi
    return rad


def calc_motor_command(distance, angle_error, drive_sign):
    # 도착 지점 근처면 정지
    if distance < STOP_DIST_PX:
        return 0, 0, "stop"

    align_rad = math.radians(ALIGN_DEG)
    # 1) 목표 방향 정렬: 회전만 수행
    if abs(angle_error) > align_rad:
        turn_pwm = clamp(TURN_PWM, 0, MAX_PWM)
        if angle_error > 0:
            # 좌회전: 한쪽 정지 + 한쪽 구동
            left = int(PIVOT_LEFT_LEFT_SIGN * turn_pwm)
            right = int(PIVOT_LEFT_RIGHT_SIGN * turn_pwm)
            return left, right, "align_left"
        # 우회전: 한쪽 정지 + 한쪽 구동
        left = int(PIVOT_RIGHT_LEFT_SIGN * turn_pwm)
        right = int(PIVOT_RIGHT_RIGHT_SIGN * turn_pwm)
        return left, right, "align_right"

    # 2) 목표를 향하면 직진 위주로 접근
    control_distance = distance + APPROACH_EXTRA_PX
    drive_pwm = clamp(BASE_PWM + KP_DISTANCE * control_distance, DRIVE_MIN_PWM, MAX_PWM)
    left = to_drive_axis(drive_pwm, drive_sign)
    right = to_drive_axis(drive_pwm, drive_sign)
    return left, right, "drive"


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    esp_link_ok = verify_esp_link_at_start(sock)
    last_link_poll = time.monotonic()

    cap, opened_index = open_camera()
    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다.")
    print(f"[카메라] 사용 인덱스: {opened_index} (설정값 CAMERA_INDEX={CAMERA_INDEX})")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
    aruco_params = cv2.aruco.DetectorParameters()
    aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

    drive_sign = FORWARD_SIGN
    prev_distance = None
    wrong_way_count = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                send_motor(sock, 0, 0)
                continue
            if MIRROR_VIEW:
                frame = cv2.flip(frame, 1)

            now = time.monotonic()
            if now - last_link_poll >= LINK_POLL_INTERVAL_SEC:
                last_link_poll = now
                esp_link_ok = poll_esp_link(sock)
                if esp_link_ok:
                    print("[연결 확인] 주기 점검: ESP8266 PONG 정상")
                else:
                    print("[연결 확인] 주기 점검: ESP8266 응답 없음")

            robot = detect_robot_pose(frame, detector)
            red_candidates = detect_red_candidates(frame)
            left_cmd, right_cmd = 0, 0

            if robot is not None and red_candidates:
                rx, ry, rtheta, rpts = robot
                robot_heading = rtheta + math.radians(MARKER_HEADING_OFFSET_DEG)
                target = select_front_target(red_candidates, rx, ry, robot_heading)
            else:
                target = None

            if target is not None and robot is not None:
                rx, ry, rtheta, rpts = robot
                tx, ty = target

                dx = tx - rx
                dy = ty - ry
                distance = math.hypot(dx, dy)
                target_theta = math.atan2(dy, dx)
                angle_error = wrap_angle(target_theta - robot_heading)
                left_cmd, right_cmd, mode = calc_motor_command(distance, angle_error, drive_sign)

                # 거리 최소화 확인: 직진 중 멀어지면 주행 부호 자동 반전
                if mode == "drive" and prev_distance is not None:
                    if distance > prev_distance + DIST_RISING_TOL_PX:
                        wrong_way_count += 1
                    else:
                        wrong_way_count = max(0, wrong_way_count - 1)

                    if wrong_way_count >= WRONG_WAY_LIMIT:
                        drive_sign *= -1
                        wrong_way_count = 0
                        print(f"[AUTO] 거리 증가 감지 -> drive_sign 반전: {drive_sign}")
                else:
                    wrong_way_count = 0

                prev_distance = distance

                cv2.polylines(frame, [rpts], True, (255, 0, 0), 2)
                cv2.circle(frame, (int(rx), int(ry)), 5, (255, 0, 0), -1)
                cv2.circle(frame, (int(tx), int(ty)), 8, (0, 0, 255), -1)
                cv2.line(frame, (int(rx), int(ry)), (int(tx), int(ty)), (0, 255, 0), 2)
                cv2.putText(
                    frame,
                    f"dist={distance:.1f}, err={math.degrees(angle_error):.1f}deg",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                cv2.putText(
                    frame,
                    f"L={int(left_cmd)} R={int(right_cmd)} mode={mode} sign={drive_sign}",
                    (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 255),
                    2,
                )
            else:
                prev_distance = None
                wrong_way_count = 0
                cv2.putText(
                    frame,
                    "Target or Robot not found",
                    (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

            send_motor(sock, left_cmd, right_cmd)

            hw_text = "HW link: OK" if esp_link_ok else "HW link: --"
            hw_color = (0, 220, 0) if esp_link_ok else (0, 180, 255)
            tw, th = cv2.getTextSize(hw_text, cv2.FONT_HERSHEY_SIMPLEX, 0.65, 2)[0]
            tx = max(10, frame.shape[1] - tw - 14)
            cv2.putText(
                frame,
                hw_text,
                (tx, 28),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                hw_color,
                2,
            )

            cv2.imshow("Robot Controller", frame)

            if cv2.waitKey(1) & 0xFF == 27:
                break

            time.sleep(LOOP_DELAY_SEC)
    finally:
        send_motor(sock, 0, 0)
        cap.release()
        cv2.destroyAllWindows()
        sock.close()


if __name__ == "__main__":
    main()
