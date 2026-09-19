from types import SimpleNamespace

from verification_core import VerificationPipeline


def ns(**values):
    return SimpleNamespace(**values)


def pose(x=1.0, y=2.0, yaw=0.0):
    import math

    return ns(
        pose=ns(
            position=ns(x=x, y=y),
            orientation=ns(x=0.0, y=0.0, z=math.sin(yaw / 2), w=math.cos(yaw / 2)),
        )
    )


class Repository:
    revision = "revision"

    def __init__(self, topology=True):
        self.node = ns(id="goal_01", name="goal")
        self.graph = ns(get_node=lambda entity_id: self.node if entity_id == "goal_01" else None)
        self._topology = topology

    def navigation_pose(self, _node):
        return (1.0, 2.0, 0.0)

    def topology_path(self, _start, _goal):
        return ["start", "goal_01"] if self._topology else []


class Geometry:
    def __init__(self, result=None):
        self.result = result or ns(valid=True, code="", message="")
        self.calls = []

    def validate_pose(self, x, y):
        self.calls.append((x, y))
        return self.result


class BarrierGeometry(Geometry):
    sequence = 7

    def __init__(self, wait_result):
        super().__init__()
        self.wait_result = wait_result
        self.waited = []

    def wait_for_update(self, sequence, timeout):
        self.waited.append((sequence, timeout))
        return self.wait_result


class Planner:
    def __init__(self, result):
        self.result = result

    def plan(self, _goal, _constraints, timeout):
        assert timeout == 10.0
        return self.result


class Mask:
    def __init__(self):
        self.cleared = 0
        self.apply_wait = None

    def apply(self, _constraints, wait=True):
        self.apply_wait = wait
        return True

    def clear(self, wait=True):
        self.cleared += 1


def resolution(**changes):
    value = dict(
        resolved=True,
        error_code="",
        message="",
        map_revision="revision",
        goal_id="goal_01",
        canonical_name="goal",
        goal_pose=pose(),
        constraints=[],
    )
    value.update(changes)
    return ns(**value)


def planned(success=True, code="", message=""):
    path = ns(poses=[pose(0, 0), pose(3, 4)])
    return ns(
        success=success,
        code=code,
        message=message,
        path=path,
        planning_time=0.125,
    )


def test_four_checks_pass_and_validation_path_is_not_returned():
    mask = Mask()
    result = VerificationPipeline(
        Repository(),
        Geometry(),
        Planner(planned()),
        mask,
        geometry_check_enabled=True,
    ).verify(resolution())

    assert result.verified
    assert [check.status for check in result.checks] == ["PASS"] * 4
    assert result.planning_length == 5.0
    assert result.planning_time == 0.125
    assert not hasattr(result, "path")
    assert mask.cleared == 1
    assert mask.apply_wait is False


def test_stale_costmap_fails_geometry_and_skips_planner():
    geometry = Geometry(ns(valid=False, code="COSTMAP_STALE", message="old"))
    result = VerificationPipeline(
        Repository(),
        geometry,
        Planner(planned()),
        Mask(),
        geometry_check_enabled=True,
    ).verify(resolution())

    assert result.error_code == "COSTMAP_STALE"
    assert [(item.name, item.status) for item in result.checks] == [
        ("entity", "PASS"),
        ("topology", "PASS"),
        ("geometry", "FAIL"),
        ("planner", "SKIPPED"),
    ]


def test_avoid_via_conflict_fails_before_planning():
    geometry = Geometry(
        ns(valid=False, code="GEOMETRY_OCCUPIED", message="occupied")
    )
    constraint = ns(type="avoid", radius=1.0, pose=pose(1.2, 2.0))
    result = VerificationPipeline(
        Repository(), geometry, Planner(planned()), Mask()
    ).verify(resolution(constraints=[constraint]))

    assert result.error_code == "CONSTRAINT_CONFLICT"
    assert geometry.calls == []


def test_disabled_geometry_occupancy_is_skipped_and_planner_still_runs():
    geometry = Geometry(
        ns(valid=False, code="GEOMETRY_OCCUPIED", message="occupied")
    )
    via = ns(type="via", radius=0.0, pose=pose(3.0, 4.0))
    result = VerificationPipeline(
        Repository(), geometry, Planner(planned()), Mask()
    ).verify(resolution(constraints=[via]))

    assert result.verified
    assert geometry.calls == []
    assert [
        (check.name, check.status, check.code) for check in result.checks
    ] == [
        ("entity", "PASS", ""),
        ("topology", "PASS", ""),
        ("geometry", "SKIPPED", "GEOMETRY_OCCUPANCY_DISABLED"),
        ("planner", "PASS", ""),
    ]
    assert "research valves open" in result.message


def test_research_valves_report_skipped_without_hiding_the_bypass():
    result = VerificationPipeline(
        Repository(topology=False),
        Geometry(ns(valid=False, code="GEOMETRY_OCCUPIED", message="occupied")),
        Planner(planned(False, "EMPTY_PATH", "empty")),
        Mask(),
        entity_check_enabled=False,
        topology_check_enabled=False,
        geometry_check_enabled=False,
        planner_check_enabled=False,
    ).verify(resolution(resolved=False))

    assert result.verified
    assert [check.status for check in result.checks] == ["SKIPPED"] * 4
    assert [check.code for check in result.checks] == [
        "ENTITY_CHECK_DISABLED",
        "TOPOLOGY_CHECK_DISABLED",
        "GEOMETRY_OCCUPANCY_DISABLED",
        "PLANNER_CHECK_DISABLED",
    ]
    assert "research valves open" in result.message


def test_empty_path_failure_is_preserved_and_mask_is_cleared():
    geometry = Geometry(
        ns(valid=False, code="GEOMETRY_OCCUPIED", message="occupied")
    )
    mask = Mask()
    result = VerificationPipeline(
        Repository(),
        geometry,
        Planner(planned(False, "EMPTY_PATH", "empty")),
        mask,
    ).verify(resolution())

    assert result.error_code == "EMPTY_PATH"
    assert geometry.calls == []
    assert [(check.name, check.status) for check in result.checks] == [
        ("entity", "PASS"),
        ("topology", "PASS"),
        ("geometry", "SKIPPED"),
        ("planner", "FAIL"),
    ]
    assert mask.cleared == 1


def test_keepout_waits_for_costmap_barrier_before_planning():
    geometry = BarrierGeometry(False)
    mask = Mask()

    result = VerificationPipeline(
        Repository(), geometry, Planner(planned()), mask
    ).verify(resolution())

    assert result.error_code == "KEEPOUT_MASK_TIMEOUT"
    assert geometry.waited == [(7, 10.0)]
    assert mask.cleared == 1
