from dt_modeling.odometry.velocity_odometer import VelocityToPose


def _v2p():
    __v2p = VelocityToPose()
    __v2p.update(0, 0, timestamp=0)
    return __v2p


def test_no_motion():
    v2p = _v2p()
    v2p.update(0, 0, timestamp=1)
    p = v2p.get_estimate()
    assert p.x == 0
    assert p.y == 0
    assert p.theta == 0


def test_straight_line():
    v2p = _v2p()
    v = 1.0
    t = 1.0
    v2p.update(v, 0, timestamp=t)
    v2p.update(0, 0, timestamp=t + 1)
    p = v2p.get_estimate()
    assert p.x == v * t
    assert p.y == 0
    assert p.theta == 0


def test_straight_line_faster():
    v2p = _v2p()
    v = 3.0
    t = 1.0
    v2p.update(v, 0, timestamp=t)
    v2p.update(0, 0, timestamp=t + 1)
    p = v2p.get_estimate()
    assert p.x == v * t
    assert p.y == 0
    assert p.theta == 0


def test_straight_line_slower():
    v2p = _v2p()
    v = 0.1
    t = 1.0
    v2p.update(v, 0, timestamp=t)
    v2p.update(0, 0, timestamp=t + 1)
    p = v2p.get_estimate()
    assert p.x == v * t
    assert p.y == 0
    assert p.theta == 0


def test_rotation_in_place():
    v2p = _v2p()
    w = 1.0
    t = 1.0
    v2p.update(0, w, timestamp=t)
    v2p.update(0, 0, timestamp=t + 1)
    p = v2p.get_estimate()
    assert p.x == 0
    assert p.y == 0
    assert p.theta == w * t
