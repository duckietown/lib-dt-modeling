from dt_modeling.kinematics.forward import ForwardKinematics

# distance between wheels
wheel_baseline: float = 0.1
# wheel radius
wheel_radius: float = 0.025


fk = ForwardKinematics(wheel_baseline, wheel_radius)


def test_no_motion():
    v, w = fk.get_chassis_speed(0, 0)
    assert v == 0
    assert w == 0


def test_straight_line():
    speed_rad_sec = 1.0
    v, w = fk.get_chassis_speed(speed_rad_sec, speed_rad_sec)
    assert v == wheel_radius * speed_rad_sec
    assert w == 0


def test_straight_line_faster():
    speed_rad_sec = 3.0
    v, w = fk.get_chassis_speed(speed_rad_sec, speed_rad_sec)
    assert v == wheel_radius * speed_rad_sec
    assert w == 0


def test_rotate_in_place_left():
    speed_rad_sec = 1.0
    v, w = fk.get_chassis_speed(-speed_rad_sec, speed_rad_sec)
    assert v == 0
    assert w == (wheel_radius * speed_rad_sec) / (wheel_baseline / 2)


def test_rotate_in_place_left_faster():
    speed_rad_sec = 3.0
    v, w = fk.get_chassis_speed(-speed_rad_sec, speed_rad_sec)
    assert v == 0
    assert w == (wheel_radius * speed_rad_sec) / (wheel_baseline / 2)


def test_rotate_in_place_right():
    speed_rad_sec = 1.0
    v, w = fk.get_chassis_speed(speed_rad_sec, -speed_rad_sec)
    assert v == 0
    assert w == -(wheel_radius * speed_rad_sec) / (wheel_baseline / 2)


def test_rotate_in_place_right_faster():
    speed_rad_sec = 3.0
    v, w = fk.get_chassis_speed(speed_rad_sec, -speed_rad_sec)
    assert v == 0
    assert w == -(wheel_radius * speed_rad_sec) / (wheel_baseline / 2)


def test_circle_left():
    speed_rad_sec = 1.0
    v, w = fk.get_chassis_speed(0, speed_rad_sec)
    assert w == (wheel_radius * speed_rad_sec) / wheel_baseline
    assert v == w * (wheel_baseline / 2)


def test_circle_left_faster():
    speed_rad_sec = 3.0
    v, w = fk.get_chassis_speed(0, speed_rad_sec)
    assert w == (wheel_radius * speed_rad_sec) / wheel_baseline
    assert v == w * (wheel_baseline / 2)


def test_circle_right():
    speed_rad_sec = 1.0
    v, w = fk.get_chassis_speed(speed_rad_sec, 0)
    assert w == -(wheel_radius * speed_rad_sec) / wheel_baseline
    assert v == -w * (wheel_baseline / 2)


def test_circle_right_faster():
    speed_rad_sec = 3.0
    v, w = fk.get_chassis_speed(speed_rad_sec, 0)
    assert w == -(wheel_radius * speed_rad_sec) / wheel_baseline
    assert v == -w * (wheel_baseline / 2)
