ROBOT_IP = ""  # 로봇 IP를 여기에 입력하거나 --robot-ip로 전달하세요.

import argparse
from pathlib import Path

import numpy as np

from kinematics import (
    RobotArm,
    from_xyz_rpy,
    inverse_transform,
    load_transform,
)
from robot_io import read_joint_angles

ROOT = Path(__file__).resolve().parent


def parse_args():
    parser = argparse.ArgumentParser(
        description="RB5-850 Eye-in-Hand visualization"
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--robot-ip")
    source.add_argument("--joints", type=float, nargs=6, metavar="DEG")

    parser.add_argument("--port", type=int, default=5001)
    parser.add_argument("--timeout", type=float, default=3.0)
    parser.add_argument(
        "--dh", type=Path, default=ROOT / "rb5_850_dh.csv"
    )
    parser.add_argument(
        "--tcp-cam", type=Path, default=ROOT / "TCP T_CAM.JSON"
    )
    parser.add_argument(
        "--cam-checkerboard",
        type=Path,
        default=ROOT / "CAM_T_CHECKERBOARD.JSON",
    )

    parser.add_argument(
        "--json-unit", choices=("m", "mm"), required=True
    )
    parser.add_argument("--tcp-cam-unit", choices=("m", "mm"))
    parser.add_argument(
        "--cam-checkerboard-unit", choices=("m", "mm")
    )

    tool = parser.add_mutually_exclusive_group(required=True)
    tool.add_argument("--dh-end-is-tcp", action="store_true")
    tool.add_argument(
        "--dh-end-to-tcp",
        type=float,
        nargs=6,
        metavar=(
            "X_MM", "Y_MM", "Z_MM",
            "RX_DEG", "RY_DEG", "RZ_DEG",
        ),
    )

    parser.add_argument("--square-mm", type=float, default=25)
    parser.add_argument(
        "--board-squares",
        type=int,
        nargs=2,
        default=(10, 7),
        metavar=("COLS", "ROWS"),
    )
    parser.add_argument(
        "--board-origin", choices=("inner", "corner"), default="inner"
    )
    parser.add_argument("--show-dh-frames", action="store_true")
    parser.add_argument("--save", type=Path)
    parser.add_argument("--no-show", action="store_true")
    args = parser.parse_args()

    if args.joints is None and not (args.robot_ip or ROBOT_IP):
        parser.error(
            "Set ROBOT_IP / --robot-ip, or supply six --joints angles"
        )
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not np.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be positive")
    if not np.isfinite(args.square_mm) or args.square_mm <= 0:
        parser.error("--square-mm must be positive")
    if min(args.board_squares) < 2:
        parser.error("--board-squares requires at least 2 squares per axis")
    return args


def main():
    args = parse_args()

    # 먼저 파일과 설정을 검증합니다.
    robot = RobotArm(args.dh)
    T_tcp_cam = load_transform(
        args.tcp_cam, args.tcp_cam_unit or args.json_unit
    )
    T_cam_board = load_transform(
        args.cam_checkerboard,
        args.cam_checkerboard_unit or args.json_unit,
    )
    T_end_tcp = (
        np.eye(4)
        if args.dh_end_is_tcp
        else from_xyz_rpy(*args.dh_end_to_tcp)
    )

    # 관절각 6개: 수동 입력 또는 로봇 실측값.
    if args.joints is not None:
        q = np.asarray(args.joints, dtype=float)
        source = "manual angles"
    else:
        ip = args.robot_ip or ROBOT_IP
        source = f"measured jnt_ang at {ip}:{args.port}"
        q = read_joint_angles(ip, args.port, args.timeout)

    # Base + 9개 DH 프레임.
    poses = robot.get_all_link_poses(q)

    T_base_tcp = poses[-1] @ T_end_tcp
    T_base_cam = T_base_tcp @ T_tcp_cam
    T_tcp_board = T_tcp_cam @ T_cam_board
    T_base_board = T_base_tcp @ T_tcp_board

    np.set_printoptions(
        precision=8, suppress=True, linewidth=120
    )
    print(f"Joint source: {source}")
    print(f"q [deg] = {q}; DH transforms = {len(robot.rows)}")
    print("Convention: p_A = T_A_B @ p_B")
    print("All translations below are in mm.")

    for name, T in (
        ("T_DH_END_TCP", T_end_tcp),
        ("T_BASE_TCP", T_base_tcp),
        ("T_BASE_CAM", T_base_cam),
        (
            "T_TCP_CHECKERBOARD (checkerboard pose in TCP)",
            T_tcp_board,
        ),
        (
            "T_CHECKERBOARD_TCP (TCP points -> checkerboard coordinates)",
            inverse_transform(T_tcp_board),
        ),
        ("T_BASE_CHECKERBOARD", T_base_board),
    ):
        print(f"\n{name}\n{T}", flush=True)

    if args.no_show and args.save is None:
        return

    if args.no_show:
        import matplotlib
        matplotlib.use("Agg")

    import matplotlib.pyplot as plt
    from visualization import plot_scene

    fig = plot_scene(
        robot,
        poses,
        T_base_tcp,
        T_base_cam,
        T_base_board,
        q,
        square_mm=args.square_mm,
        board_squares=args.board_squares,
        board_origin=args.board_origin,
        show_dh_frames=args.show_dh_frames,
    )

    # --------------------------------------------------------
    # 좌표축이 체커보드에 가려지지 않도록 표시 순서 조정
    # --------------------------------------------------------
    import matplotlib.patheffects as pe
    from mpl_toolkits.mplot3d.art3d import (
        Line3DCollection,
        Path3DCollection,
        Poly3DCollection,
    )

    ax = fig.axes[0]

    # 지정한 zorder가 적용되도록 자동 깊이 정렬을 끕니다.
    ax.computed_zorder = False

    for artist in ax.collections:
        if isinstance(artist, Poly3DCollection):
            # 체커보드 면을 반투명하게 표시합니다.
            artist.set_alpha(0.25)
            artist.set_zorder(1)

        elif isinstance(artist, Line3DCollection):
            # draw_frame()의 quiver 좌표축입니다.
            artist.set_zorder(20)
            artist.set_linewidths(2.5)
            artist.set_path_effects([
                pe.Stroke(linewidth=5, foreground="white"),
                pe.Normal(),
            ])

        elif isinstance(artist, Path3DCollection):
            # 좌표계 원점 마커도 앞으로 표시합니다.
            artist.set_zorder(21)
            artist.set_depthshade(False)

    for text in ax.texts:
        text.set_zorder(30)
        text.set_bbox({
            "facecolor": "white",
            "edgecolor": "none",
            "alpha": 0.8,
            "pad": 1.5,
        })

    if args.save:
        args.save.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.save, dpi=160)
        print(f"\nSaved: {args.save.resolve()}")

    if args.no_show:
        plt.close(fig)
    else:
        plt.show()


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, RuntimeError) as exc:
        raise SystemExit(f"ERROR: {exc}")
