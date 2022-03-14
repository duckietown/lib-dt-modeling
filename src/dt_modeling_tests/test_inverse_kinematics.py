from dt_modeling.kinematics.inverse import InverseKinematics

# distance between wheels
wheel_baseline: float = 0.1
# wheel radius
wheel_radius: float = 0.025

# IK parameters
gain: float = 1.0
trim: float = 0.0
k: float = 27.0
limit: float = 1.0
v_max: float = 1.0
omega_max: float = 8.0

ik = InverseKinematics(
    wheel_baseline=wheel_baseline,
    wheel_radius=wheel_radius,
    gain=gain,
    trim=trim,
    k=k,
    limit=limit,
    v_max=v_max,
    omega_max=omega_max,
)


def test_no_motion():
    wl, wr = ik.get_wheels_speed(0, 0)
    assert wl == 0
    assert wr == 0


def test_straight_line():
    v = 0.1
    wl, wr = ik.get_wheels_speed(v, 0)
    assert wl == v / wheel_radius
    assert wr == v / wheel_radius


def test_straight_line_faster():
    v = 0.9
    wl, wr = ik.get_wheels_speed(v, 0)
    assert wl == v / wheel_radius
    assert wr == v / wheel_radius


def test_straight_line_above_limit():
    v = 3.0
    wl, wr = ik.get_wheels_speed(v, 0)
    assert wl == v_max / wheel_radius
    assert wr == v_max / wheel_radius
