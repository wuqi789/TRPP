import ast
from pathlib import Path


SOURCE_PATH = Path(__file__).resolve().parents[1] / "scout_avoidance.py"


def _source_tree():
    return ast.parse(SOURCE_PATH.read_text(encoding="utf-8"))


def _function(tree, name):
    return next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )


def test_piper_command_subscription_keeps_only_the_latest_target():
    function = _function(_source_tree(), "create_arm_subscription")
    subscriptions = [
        node
        for node in ast.walk(function)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "create_subscription"
        and len(node.args) >= 4
        and isinstance(node.args[1], ast.Constant)
        and node.args[1].value == "/isaac_joint_command"
    ]
    assert len(subscriptions) == 1
    depth = subscriptions[0].args[3]
    assert isinstance(depth, ast.Constant)
    assert depth.value == 1


def test_piper_rigid_bodies_disable_gravity():
    function = _function(_source_tree(), "configure_arm_drives")
    calls = [
        node
        for node in ast.walk(function)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "CreateDisableGravityAttr"
    ]
    assert len(calls) == 1
    parent_source = ast.unparse(function)
    assert ".CreateDisableGravityAttr().Set(True)" in parent_source


def test_piper_angular_damping_is_tuned_for_incremental_targets():
    function = _function(_source_tree(), "configure_arm_drives")
    parent_source = ast.unparse(function)
    assert (
        "drive.CreateDampingAttr().Set(float(ARM_KD[index] * ARM_DAMPING_SCALE))"
        in parent_source
    )
    assert "ARM_KD[index] * unit_scale" not in parent_source

    assignments = {
        node.targets[0].id: node.value.value
        for node in _source_tree().body
        if isinstance(node, ast.Assign)
        and len(node.targets) == 1
        and isinstance(node.targets[0], ast.Name)
        and isinstance(node.value, ast.Constant)
    }
    assert assignments["ARM_DAMPING_SCALE"] == 0.12
