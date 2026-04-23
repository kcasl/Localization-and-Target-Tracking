"""
모터 전진 주행 테스트 (노트북 → ESP8266 UDP).

pc_robot_controller.py 와 동일하게 "left,right" 형식으로 전송합니다.
ESP 펌웨어가 PING/PONG을 지원하면 연결 확인 후 전진합니다.

예:
  python src/motor_forward_test.py
  python src/motor_forward_test.py --esp 192.168.0.50 --pwm 140 --seconds 2
  python src/motor_forward_test.py --skip-ping
  python src/motor_forward_test.py --invert-right
"""

from __future__ import annotations

import argparse
import socket
import sys
import time

# pc_robot_controller.py 의 ESP_IP / ESP_PORT 와 맞출 것
DEFAULT_ESP_IP = "10.31.216.22"
DEFAULT_ESP_PORT = 4210

LINK_PING = b"PING"
LINK_PONG_PREFIX = b"PONG"
LINK_RETRIES = 3
LINK_TIMEOUT_SEC = 2.0


def send_motor(sock: socket.socket, esp_ip: str, esp_port: int, left: int, right: int) -> None:
    msg = f"{int(left)},{int(right)}".encode("utf-8")
    sock.sendto(msg, (esp_ip, esp_port))


def verify_esp(sock: socket.socket, esp_ip: str, esp_port: int) -> bool:
    print(
        f"[연결 확인] PING → {esp_ip}:{esp_port} "
        f"(최대 {LINK_RETRIES}회, 대기 {LINK_TIMEOUT_SEC}s)"
    )
    for attempt in range(1, LINK_RETRIES + 1):
        try:
            sock.sendto(LINK_PING, (esp_ip, esp_port))
            sock.settimeout(LINK_TIMEOUT_SEC)
            data, addr = sock.recvfrom(256)
        except socket.timeout:
            print(f"[연결 확인] 시도 {attempt}/{LINK_RETRIES}: PONG 없음")
            continue
        finally:
            sock.settimeout(None)

        if data.startswith(LINK_PONG_PREFIX) and addr[0] == esp_ip:
            print(f"[연결 확인] 성공: {addr[0]}:{addr[1]} 에서 PONG 수신")
            return True
        print(
            f"[연결 확인] 시도 {attempt}/{LINK_RETRIES}: "
            f"비정상 응답 from={addr[0]}:{addr[1]} data={data!r}"
        )

    print(
        "[연결 확인] 실패. ESP IP/포트, Wi-Fi, 방화벽, 펌웨어(PING→PONG)를 확인하세요."
    )
    return False


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="모터 전진 테스트 (UDP)")
    p.add_argument("--esp", default=DEFAULT_ESP_IP, help=f"ESP8266 IP (기본: {DEFAULT_ESP_IP})")
    p.add_argument("--port", type=int, default=DEFAULT_ESP_PORT, help=f"UDP 포트 (기본: {DEFAULT_ESP_PORT})")
    p.add_argument("--pwm", type=int, default=120, help="좌우 동일 전진 PWM 1~255 (기본: 120)")
    p.add_argument("--seconds", type=float, default=3.0, help="전진 유지 시간(초) (기본: 3)")
    p.add_argument("--hz", type=float, default=20.0, help="명령 전송 주기(Hz), 타임아웃 방지 (기본: 20)")
    p.add_argument("--skip-ping", action="store_true", help="PING 생략, 바로 모터 명령만 전송")
    p.add_argument(
        "--invert-right",
        action="store_true",
        help="우측 모터만 반대 방향으로 보냄 (배선이 반대일 때)",
    )
    p.add_argument(
        "--invert-left",
        action="store_true",
        help="좌측 모터만 반대 방향으로 보냄",
    )
    p.add_argument(
        "--require-link",
        action="store_true",
        help="PING 실패 시 전진하지 않고 종료(exit 1)",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    pwm = max(1, min(255, args.pwm))
    period = 1.0 / max(1.0, args.hz)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        if not args.skip_ping:
            ok = verify_esp(sock, args.esp, args.port)
            if args.require_link and not ok:
                return 1
            if not ok:
                print("[안내] 연결 확인 실패했지만 --require-link 없음 → 전진 테스트는 계속합니다.")

        left = pwm
        right = pwm
        if args.invert_left:
            left = -left
        if args.invert_right:
            right = -right

        print(
            f"[전진] L={left} R={right} 로 {args.seconds}s 동안 전송 "
            f"(주기 약 {args.hz:.0f}Hz, 종료 시 0,0)"
        )
        t0 = time.monotonic()
        t_end = t0 + args.seconds
        next_status = t0

        while True:
            now = time.monotonic()
            if now >= t_end:
                break

            send_motor(sock, args.esp, args.port, left, right)

            if now >= next_status:
                elapsed = now - t0
                print(f"  … 작동 중  L={left} R={right}  (경과 {elapsed:.1f}s / {args.seconds:.1f}s)")
                next_status += 0.5

            time.sleep(period)

        send_motor(sock, args.esp, args.port, 0, 0)
        print("[전진] 정지 명령 0,0 전송 완료. 바퀴가 멈췄는지 확인하세요.")
        return 0
    except KeyboardInterrupt:
        print("\n[전진] 중단 → 정지")
        send_motor(sock, args.esp, args.port, 0, 0)
        return 130
    finally:
        sock.close()


if __name__ == "__main__":
    sys.exit(main())
