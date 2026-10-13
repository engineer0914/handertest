import json
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


# ============================================================
# 설정
# ============================================================

CHECKERBOARD = (7, 5)   # 내부 코너 개수. 실제 체커 칸은 8 x 6.
SQUARE_SIZE = 0.030     # 한 칸 크기 [m]
AXIS_LENGTH = 0.090     # 표시할 좌표축 길이 [m]

# True이면 스페이스바 입력 시 JSON 파일도 저장합니다.
SAVE_JSON_ON_SPACE = False

OUTPUT_JSON = (
    Path(__file__).resolve().parent / "CAM_T_CHECKERBOARD.JSON"
)

WINDOW = "D435 Checkerboard Pose"


# ============================================================
# 검출 코너 -> 왜곡 보정된 정규화 영상 좌표
# ============================================================

def normalized_corners(corners, intr):
    pixels = corners.reshape(-1, 2)

    if intr.model == rs.distortion.none or not np.any(intr.coeffs):
        return (
            (pixels.astype(np.float64) - [intr.ppx, intr.ppy])
            / [intr.fx, intr.fy]
        ).reshape(-1, 1, 2)

    # RealSense의 왜곡 모델을 적용합니다.
    # 여기서 1.0은 광선 방향을 얻기 위한 값이며,
    # 실제 깊이 센서 측정값을 사용하는 것은 아닙니다.
    rays = [
        rs.rs2_deproject_pixel_to_point(
            intr, [float(u), float(v)], 1.0
        )
        for u, v in pixels
    ]

    return (
        np.asarray(rays, dtype=np.float64)[:, :2]
        .reshape(-1, 1, 2)
    )


# ============================================================
# 체커보드 XYZ 축 표시
# ============================================================

def draw_axes(image, R, tvec, intr):
    board_points = np.array([
        [0, 0, 0],
        [AXIS_LENGTH, 0, 0],
        [0, AXIS_LENGTH, 0],
        [0, 0, AXIS_LENGTH],
    ], dtype=np.float64)

    camera_points = board_points @ R.T + tvec.reshape(1, 3)

    if np.any(camera_points[:, 2] <= 1e-6):
        return

    pixels = np.array([
        rs.rs2_project_point_to_pixel(intr, point.tolist())
        for point in camera_points
    ])

    if (
        not np.isfinite(pixels).all()
        or np.max(np.abs(pixels)) > 1e7
    ):
        return

    pixels = np.rint(pixels).astype(np.int32)
    origin = tuple(map(int, pixels[0]))

    cv2.circle(image, origin, 5, (255, 255, 255), -1)

    # OpenCV 색상 순서: BGR
    colors = (
        (0, 0, 255),   # X: 빨강
        (0, 255, 0),   # Y: 초록
        (255, 0, 0),   # Z: 파랑
    )

    for endpoint, label, color in zip(
        pixels[1:], ("X", "Y", "Z"), colors
    ):
        endpoint = tuple(map(int, endpoint))

        cv2.arrowedLine(
            image, origin, endpoint, color,
            3, cv2.LINE_AA, tipLength=0.12,
        )
        cv2.putText(
            image, label,
            (endpoint[0] + 5, endpoint[1] - 5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7, color, 2, cv2.LINE_AA,
        )


# ============================================================
# 4x4 행렬을 유효한 JSON 형식으로 출력
# ============================================================

def matrix_to_json(T):
    rows = np.round(T, 8).tolist()

    return (
        "[\n"
        + ",\n".join("  " + json.dumps(row) for row in rows)
        + "\n]"
    )


# ============================================================
# 실행
# ============================================================

def main():
    # 체커보드 원점: 검출된 첫 내부 코너.
    # X: 코너의 열 방향, Y: 행 방향, Z: 보드에 수직인 방향.
    object_points = np.zeros(
        (CHECKERBOARD[0] * CHECKERBOARD[1], 3),
        dtype=np.float64,
    )

    object_points[:, :2] = (
        np.mgrid[
            0:CHECKERBOARD[0],
            0:CHECKERBOARD[1],
        ].T.reshape(-1, 2) * SQUARE_SIZE
    )

    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )

    pipeline = rs.pipeline()
    config = rs.config()

    config.enable_stream(
        rs.stream.color,
        640,
        480,
        rs.format.bgr8,
        30,
    )

    started = False

    try:
        profile = pipeline.start(config)
        started = True

        color_stream = (
            profile.get_stream(rs.stream.color)
            .as_video_stream_profile()
        )
        intr = color_stream.get_intrinsics()

        supported_models = (
            rs.distortion.none,
            rs.distortion.brown_conrady,
            rs.distortion.inverse_brown_conrady,
        )

        if (
            intr.model not in supported_models
            and np.any(intr.coeffs)
        ):
            raise RuntimeError(
                f"Unsupported color distortion model: {intr.model}"
            )

        print("========== Camera Intrinsic ==========")
        print(f"Size  : {intr.width} x {intr.height}")
        print(f"fx/fy : {intr.fx}, {intr.fy}")
        print(f"cx/cy : {intr.ppx}, {intr.ppy}")
        print(f"Model : {intr.model}")
        print(f"Dist  : {intr.coeffs}")
        print("======================================")
        print(f"Checkerboard: {CHECKERBOARD}")
        print(f"Square size: {SQUARE_SIZE} m")
        print("Camera axes: X right, Y down, Z forward")
        print("p_cam = T_CAM_CHECKERBOARD @ p_checkerboard")
        print("[SPACE] print current matrix")
        print("[Q / ESC] quit")
        print(f"Save JSON on SPACE: {SAVE_JSON_ON_SPACE}")
        print("Click the video window before pressing keys.", flush=True)

        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)

        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            image = np.asanyarray(color_frame.get_data())
            display = image.copy()
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # 매 프레임 초기화하여 이전 검출 좌표가 출력되지 않게 합니다.
            current_T = None
            status = "NOT DETECTED"
            status_color = (0, 0, 255)

            found, corners = cv2.findChessboardCorners(
                gray,
                CHECKERBOARD,
                cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE,
            )

            if found:
                corners = cv2.cornerSubPix(
                    gray,
                    corners,
                    (5, 5),
                    (-1, -1),
                    criteria,
                )

                cv2.drawChessboardCorners(
                    display,
                    CHECKERBOARD,
                    corners,
                    True,
                )

                image_points = normalized_corners(corners, intr)

                # 정규화 및 왜곡 보정된 좌표이므로 K=I, 왜곡=0.
                ok, rvec, tvec = cv2.solvePnP(
                    object_points,
                    image_points,
                    np.eye(3, dtype=np.float64),
                    np.zeros(5, dtype=np.float64),
                    flags=cv2.SOLVEPNP_ITERATIVE,
                )

                status = "POSE ESTIMATION FAILED"

                if (
                    ok
                    and np.isfinite(rvec).all()
                    and np.isfinite(tvec).all()
                ):
                    R, _ = cv2.Rodrigues(rvec)

                    camera_points = (
                        object_points @ R.T + tvec.reshape(1, 3)
                    )

                    if np.all(camera_points[:, 2] > 0):
                        current_T = np.eye(4, dtype=np.float64)
                        current_T[:3, :3] = R
                        current_T[:3, 3] = tvec.ravel()

                        draw_axes(display, R, tvec, intr)

                        status = "DETECTED"
                        status_color = (0, 255, 0)

                        x, y, z = tvec.ravel()

                        cv2.putText(
                            display,
                            f"x={x:.4f}  y={y:.4f}  z={z:.4f} m",
                            (15, 65),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6,
                            (255, 255, 255),
                            2,
                            cv2.LINE_AA,
                        )

            cv2.putText(
                display,
                status,
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                status_color,
                2,
            )

            cv2.putText(
                display,
                "SPACE: print pose | Q/ESC: quit",
                (15, 460),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            cv2.imshow(WINDOW, display)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), ord("Q"), 27):
                break

            if key == 32:  # 스페이스바
                if current_T is None:
                    print(
                        "[SPACE] No valid checkerboard pose "
                        "in this frame.",
                        flush=True,
                    )
                else:
                    matrix_text = matrix_to_json(current_T)

                    print(
                        "\nT_CAM_CHECKERBOARD [translation: m]"
                        f" | frame={color_frame.get_frame_number()}"
                        f"\n{matrix_text}",
                        flush=True,
                    )

                    if SAVE_JSON_ON_SPACE:
                        OUTPUT_JSON.write_text(
                            matrix_text + "\n",
                            encoding="utf-8",
                        )
                        print(f"[SAVE] {OUTPUT_JSON}", flush=True)

            if cv2.getWindowProperty(
                WINDOW, cv2.WND_PROP_VISIBLE
            ) < 1:
                break

    finally:
        try:
            if started:
                pipeline.stop()
        finally:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nStopped.")
    except (RuntimeError, ValueError, cv2.error) as exc:
        raise SystemExit(f"ERROR: {exc}")
