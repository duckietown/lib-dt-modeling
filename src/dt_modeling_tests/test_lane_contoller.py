import numpy as np

from dt_modeling.lane_controller import ILaneController, PIDLaneController

v_bar: float = 0.2
k_d: float = -6
k_theta: float = -5
k_Id: float = -0.3
k_Iphi: float = 0.0
d_thres: float = 0.25
phi_thres: float = np.deg2rad(30)
d_offset: float = 0.0
phi_offset: float = 0.0
omega_ff: float = 0
# TODO: these should match those from the lane filter
d_resolution: float = 0.02
phi_resolution: float = np.deg2rad(5)


def _controller(**kwargs):
    args = {
        "v_bar": v_bar,
        "k_d": k_d,
        "k_theta": k_theta,
        "k_Id": k_Id,
        "k_Iphi": k_Iphi,
        "d_thres": d_thres,
        "phi_thres": phi_thres,
        "d_offset": d_offset,
        "phi_offset": phi_offset,
        "d_resolution": d_resolution,
        "phi_resolution": phi_resolution,
        "omega_ff": omega_ff,
    }
    args.update(kwargs)
    return PIDLaneController(**args)


def test_no_error_single():
    controller: ILaneController = _controller()
    controller.update(0.0, 0.0)
    v, w = controller.compute_commands()
    assert v == v_bar
    assert w == 0.0


def test_no_error_multiple():
    controller: ILaneController = _controller()
    for t in range(100):
        controller.update(0.0, 0.0, timestamp=t)
        v, w = controller.compute_commands()
        assert v == v_bar
        assert w == 0.0


def test_lateral_error_single():
    controller: ILaneController = _controller()
    d_err = 0.2
    controller.update(d_err, 0.0)
    v, w = controller.compute_commands()
    assert v == v_bar
    assert equal(w, d_err * k_d)


def test_lateral_error_multiple():
    controller: ILaneController = _controller()
    d_err = 0.2
    expected_w = [-1.2, -1.11, -1.17, -1.23]
    for t, w_exp in enumerate(expected_w):
        controller.update(d_err, 0.0, timestamp=t)
        v, w = controller.compute_commands()
        assert v == v_bar
        assert equal(w, w_exp)


def test_accounted_lateral_error_single():
    _d_offset = 0.2
    controller: ILaneController = _controller(d_offset=_d_offset)
    controller.update(_d_offset, 0.0)
    v, w = controller.compute_commands()
    assert v == v_bar
    assert w == 0.0


def test_accounted_lateral_error_multiple():
    _d_offset = 0.2
    controller: ILaneController = _controller(d_offset=_d_offset)
    for t in range(100):
        controller.update(_d_offset, 0.0, timestamp=t)
        v, w = controller.compute_commands()
        assert v == v_bar
        assert w == 0.0


def equal(a, b):
    print(a, "?=", b)
    return np.allclose(a, b)
