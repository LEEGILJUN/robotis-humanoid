"""End-to-end checks of the tools against the simulated bus (no hardware needed)."""
import importlib

import pytest


@pytest.fixture(autouse=True)
def tmp_dirs(tmp_path, monkeypatch):
    monkeypatch.setenv("HUMANOID_LOG_DIR", str(tmp_path / "logs"))
    monkeypatch.setenv("HUMANOID_POSE_DIR", str(tmp_path / "poses"))
    import humanoid.robot as robot
    importlib.reload(robot)
    for name in ("tools.scan", "tools.motor_test", "tools.pose", "tools.gripper_test"):
        importlib.reload(importlib.import_module(name))


def test_scan_healthy_robot():
    from tools import scan
    r = scan.main(["--port", "sim"])
    assert r["missing"] == []
    assert r["unexpected"] == []
    v = {row["id"]: row["verdict"] for row in r["rows"]}
    assert v.pop(19) == "WARN" and v.pop(20) == "WARN"  # grippers in wheel mode
    assert set(v.values()) == {"PASS"}


def _sim_spin(module_main, argv):
    """Run a tool against a fresh SimBus and return how far the grippers spun."""
    import humanoid.robot as robot
    buses = []
    real_open = robot.open_bus

    def capture(args, cfg):
        bus = real_open(args, cfg)
        buses.append(bus)
        return bus

    mod = importlib.import_module(module_main)
    mod.open_bus = capture
    try:
        result = mod.main(argv)
    finally:
        mod.open_bus = real_open
    bus = buses[0]
    return result, {i: getattr(bus.motors[i], "spun_deg", 0.0) for i in (19, 20)}


def test_motor_test_never_spins_wheel_mode_grippers():
    rows, spun = _sim_spin("tools.motor_test", ["--port", "sim", "-y"])
    assert spun == {19: 0.0, 20: 0.0}
    grip = {r["id"]: r for r in rows if r["id"] in (19, 20)}
    assert all("motion" not in r for r in grip.values())
    assert all(r["verdict"] == "WARN" for r in grip.values())


def test_pose_play_excludes_wheel_mode_grippers():
    from tools import pose
    rec = pose.main(["--port", "sim", "-y", "record", "home"])
    assert 19 not in rec["joints"] and 20 not in rec["joints"]
    _, spun = _sim_spin("tools.pose", ["--port", "sim", "-y", "play", "home", "--duration", "1", "--hold", "1"])
    assert spun == {19: 0.0, 20: 0.0}


def test_scan_reports_faults():
    from tools import scan
    r = scan.main(["--port", "sim", "--sim-faults"])
    by_id = {row["id"]: row for row in r["rows"]}
    assert r["missing"] == [11]
    assert by_id[16]["verdict"] == "FAIL"      # overheating error bit
    assert by_id[3]["verdict"] == "WARN"       # torque limit 0
    assert by_id[1]["verdict"] == "PASS"


def test_scan_other_baud_finds_nothing():
    from tools import scan
    r = scan.main(["--port", "sim", "--baud", "57600"])
    assert r["found"] == {}


def test_motor_test_detects_each_fault():
    from tools import motor_test
    rows = motor_test.main(["--port", "sim", "--sim-faults", "-y"])
    v = {row["id"]: row["verdict"] for row in rows}
    assert v[11] == "FAIL"   # missing
    assert v[14] == "FAIL"   # stuck -> overload / not reached
    assert v[16] == "FAIL"   # hot -> motion skipped, FAIL kept
    assert v[5] == "WARN"    # potentiometer jitter
    assert v[3] == "WARN"    # torque limit was 0, but moves after restore
    assert v[1] == "PASS"
    stuck = next(r for r in rows if r["id"] == 14)
    assert any("overload" in s["errors"] for s in stuck["motion"]["steps"])


def test_motor_test_healthy_moves_and_returns():
    from tools import motor_test
    rows = motor_test.main(["--port", "sim", "--ids", "13", "-y"])
    m = rows[0]["motion"]
    assert rows[0]["verdict"] == "PASS"
    assert len(m["steps"]) == 3
    assert all(s["error_deg"] <= 1.0 for s in m["steps"])


def test_pose_record_and_play():
    from tools import pose
    rec = pose.main(["--port", "sim", "-y", "record", "home"])
    assert len(rec["joints"]) == 18  # wheel-mode grippers excluded
    res = pose.main(["--port", "sim", "-y", "play", "home", "--duration", "1", "--hold", "1"])
    assert set(res["stats"]) == set(rec["joints"])
    assert all(s["max_error_deg"] <= 1.0 for s in res["stats"].values())


def test_gripper_test_finds_stops_and_direction():
    from tools import gripper_test
    res = {r["id"]: r for r in gripper_test.main(["--port", "sim", "-y"])}
    assert res[19]["open_direction"] == "CCW" and res[20]["open_direction"] == "CW"
    for r in res.values():
        assert r["verdict"] == "PASS"
        assert abs(r["open_travel_deg"] - 170) < 5 and abs(r["close_travel_deg"] - 170) < 5
    assert res[19]["dead_zone_crossed"] is True
