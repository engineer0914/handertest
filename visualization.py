"""Matplotlib 3D. 모든 위치와 길이는 mm."""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection


def transform_points(T, points):
    points = np.asarray(points, dtype=float)
    return points @ T[:3, :3].T + T[:3, 3]


def draw_frame(ax, T, label, scale=80):
    origin = T[:3, 3]
    for column, color in enumerate(("red", "green", "blue")):
        ax.quiver(
            *origin,
            *(T[:3, column] * scale),
            color=color,
            linewidth=1.8,
            arrow_length_ratio=0.15,
        )
    ax.scatter(*origin, color="black", s=25)
    ax.text(*(origin + scale * 0.08), label, fontsize=8)
    return np.vstack([
        origin,
        origin + T[:3, :3].T * scale,
    ])


def plot_scene(
    robot, poses, T_base_tcp, T_base_cam, T_base_board, q,
    square_mm=25, board_squares=(10, 7), board_origin="inner",
    show_dh_frames=False,
):
    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection="3d")

    points = np.array([T[:3, 3] for T in poses])
    extents = [points]
    ax.plot(
        *points.T, "o-", color="0.3", linewidth=3, markersize=4,
        label=f"DH chain ({len(robot.rows)} transforms)",
    )

    for index, row in enumerate(robot.rows):
        joint = row[0]
        if joint is not None:
            ax.text(
                *points[index], f"J{joint + 1}",
                color="0.3", fontsize=8,
            )

    if show_dh_frames:
        for index, T in enumerate(poses[1:], 1):
            extents.append(
                draw_frame(ax, T, f"DH {index}", scale=35)
            )

    for T, name in (
        (poses[0], "Base"),
        (T_base_tcp, "TCP"),
        (T_base_cam, "Camera"),
        (T_base_board, "Checkerboard"),
    ):
        extents.append(draw_frame(ax, T, name))

    tool_link = np.array([
        poses[-1][:3, 3], T_base_tcp[:3, 3]
    ])
    ax.plot(
        *tool_link.T, color="cyan", linewidth=3,
        label="DH end - TCP",
    )

    mount = np.array([
        T_base_tcp[:3, 3], T_base_cam[:3, 3]
    ])
    ax.plot(
        *mount.T, "--", color="purple", linewidth=2,
        label="TCP - Camera",
    )
    sight = np.array([
        T_base_cam[:3, 3], T_base_board[:3, 3]
    ])
    ax.plot(
        *sight.T, "--", color="orange",
        label="Camera - Checkerboard",
    )

    # 카메라 +Z 방향의 개략적 프러스텀.
    # 실제 카메라 내부 파라미터로 계산한 시야각은 아닙니다.
    corners = transform_points(T_base_cam, [
        [-35, -25, 100], [35, -25, 100],
        [35, 25, 100], [-35, 25, 100],
    ])
    ring = np.vstack([corners, corners[0]])
    ax.plot(
        *ring.T, color="purple", alpha=0.6,
        label="Camera +Z (schematic)",
    )
    for corner in corners:
        edge = np.array([T_base_cam[:3, 3], corner])
        ax.plot(*edge.T, color="purple", alpha=0.4)
    extents.append(corners)

    # inner: 첫 내부 코너가 체커보드 좌표계 원점.
    cols, rows = board_squares
    offset = -1 if board_origin == "inner" else 0
    polygons, colors = [], []

    for row in range(rows):
        for col in range(cols):
            x = (col + offset) * square_mm
            y = (row + offset) * square_mm
            local = [
                [x, y, 0],
                [x + square_mm, y, 0],
                [x + square_mm, y + square_mm, 0],
                [x, y + square_mm, 0],
            ]
            polygons.append(
                transform_points(T_base_board, local)
            )
            colors.append(
                "white" if (row + col) % 2 == 0 else "0.1"
            )

    ax.add_collection3d(Poly3DCollection(
        polygons,
        facecolors=colors,
        edgecolors="0.4",
        linewidths=0.3,
        alpha=0.85,
    ))
    extents.append(np.vstack(polygons))

    # 모든 대상이 보이도록 범위 계산. XYZ 축척은 동일.
    cloud = np.vstack(extents)
    lower, upper = cloud.min(axis=0), cloud.max(axis=0)
    center = (lower + upper) / 2
    half = max(float(np.max(upper - lower)) / 2, 100) * 1.15

    ax.set_xlim(center[0] - half, center[0] + half)
    ax.set_ylim(center[1] - half, center[1] + half)
    ax.set_zlim(center[2] - half, center[2] + half)
    ax.set_box_aspect((1, 1, 1))
    ax.set(
        xlabel="X [mm]",
        ylabel="Y [mm]",
        zlabel="Z [mm]",
        title="RB5-850 Eye-in-Hand | X=red, Y=green, Z=blue",
    )
    ax.view_init(elev=25, azim=45)
    ax.legend(
        loc="upper left", bbox_to_anchor=(1.0, 1.0), fontsize=8
    )
    fig.text(
        0.05, 0.03,
        "Measured/input q [deg]: " +
        ", ".join(f"{angle:.2f}" for angle in q),
        fontsize=9,
    )
    fig.subplots_adjust(
        left=0.03, right=0.78, bottom=0.08, top=0.93
    )
    return fig
