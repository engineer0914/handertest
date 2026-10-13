"""Rainbow Robotics 데이터 채널에서 관절 실측값 읽기."""
import numpy as np


def read_joint_angles(robot_ip, port=5001, timeout=3.0):
    if not np.isfinite(timeout) or timeout <= 0:
        raise ValueError("Data request timeout must be positive")
    try:
        import rbpodo as rb
    except ImportError as exc:
        raise RuntimeError(
            "Robot mode requires rbpodo: "
            "python -m pip install -r requirements-robot.txt"
        ) from exc

    try:
        channel = rb.CobotData(robot_ip, port)
        state = channel.request_data(timeout=timeout)
        if state is None:
            raise TimeoutError(
                f"No robot state received within {timeout}s"
            )
        q = np.array(
            state.sdata.jnt_ang, dtype=float, copy=True
        ).reshape(-1)
    except Exception as exc:
        raise RuntimeError(
            f"Cannot read robot at {robot_ip}:{port}: {exc}"
        ) from exc

    if q.shape != (6,) or not np.isfinite(q).all():
        raise RuntimeError(
            "Robot returned invalid measured joint angles"
        )
    return q
