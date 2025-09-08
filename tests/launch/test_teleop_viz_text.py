import pathlib


def test_teleop_viz_contains_expected_args():
    p = pathlib.Path("launch/teleop_viz.launch.py")
    text = p.read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("use_teleop"' in text
    assert '"port": LaunchConfiguration("ws_port")' in text
    assert '"address": LaunchConfiguration("ws_address")' in text
