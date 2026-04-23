"""
마커 이미지(예: 프로젝트 루트의 marker.png)가 OpenCV ArUco 사전 중 무엇에 해당하는지 확인합니다.

사용 예:
  python src/identify_marker.py ../marker.png
  python src/identify_marker.py c:/Project/hdw/marker.png
"""

import argparse
import sys

import cv2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "image",
        nargs="?",
        default="marker.png",
        help="마커 PNG/JPG 경로 (기본: 현재 폴더의 marker.png)",
    )
    args = parser.parse_args()

    img = cv2.imread(args.image)
    if img is None:
        print(f"이미지를 열 수 없습니다: {args.image}", file=sys.stderr)
        sys.exit(1)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    dict_names = [
        name
        for name in dir(cv2.aruco)
        if name.startswith("DICT_") and name not in ("DICT_ARUCO_ORIGINAL",)
    ]
    dict_names.sort()

    # 같은 정수 ID를 가진 사전 별칭(예: 36H12 / 36h12)은 한 번만 시도
    dict_id_to_name = {}
    for name in dict_names:
        try:
            d = getattr(cv2.aruco, name)
            if not isinstance(d, int):
                continue
        except Exception:
            continue
        if d not in dict_id_to_name:
            dict_id_to_name[d] = name

    hits = []
    for d, name in dict_id_to_name.items():
        ad = cv2.aruco.getPredefinedDictionary(d)
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        det = cv2.aruco.ArucoDetector(ad, params)

        for label, mat in (("gray", gray), ("bw", bw)):
            corners, ids, _ = det.detectMarkers(mat)
            if ids is None:
                continue
            for mid in ids.flatten().tolist():
                hits.append((label, name, mid))

    if not hits:
        print("어떤 사전에서도 마커를 찾지 못했습니다.")
        sys.exit(2)

    print("감지 결과 (label, dictionary, marker_id):")
    seen = set()
    for row in hits:
        if row in seen:
            continue
        seen.add(row)
        print(f"  {row[0]:4s}  {row[1]}  id={row[2]}")

    first = hits[0]
    print("\npc_robot_controller.py 에 반영:")
    print(f"  ARUCO_DICT = cv2.aruco.{first[1]}")
    print(f"  MARKER_ID = {first[2]}")


if __name__ == "__main__":
    main()
