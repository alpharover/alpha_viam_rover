import json
import pathlib

import yaml
from jsonschema import validate
import pytest


ROOT = pathlib.Path(__file__).resolve().parents[2]


def _validate_yaml(yaml_path: str, schema_path: str) -> None:
    schema = json.loads((ROOT / schema_path).read_text(encoding="utf-8"))
    data = yaml.safe_load((ROOT / yaml_path).read_text(encoding="utf-8"))
    validate(instance=data, schema=schema)


@pytest.mark.unit
def test_topics_yaml_valid():
    _validate_yaml("configs/topics.yaml", "configs/schemas/topics.schema.json")


@pytest.mark.unit
def test_network_yaml_valid():
    _validate_yaml("configs/network.yaml", "configs/schemas/network.schema.json")


@pytest.mark.unit
def test_diff_drive_yaml_valid():
    _validate_yaml("configs/diff_drive.yaml", "configs/schemas/diff_drive.schema.json")


@pytest.mark.unit
def test_ekf_yaml_valid():
    _validate_yaml("configs/ekf.yaml", "configs/schemas/ekf.schema.json")


@pytest.mark.unit
def test_imu_yaml_valid():
    _validate_yaml("configs/imu.yaml", "configs/schemas/imu.schema.json")


@pytest.mark.unit
def test_ydlidar_yaml_valid():
    _validate_yaml("configs/ydlidar_g4.yaml", "configs/schemas/ydlidar_g4.schema.json")


@pytest.mark.unit
def test_power_yaml_valid():
    _validate_yaml("configs/power.yaml", "configs/schemas/power.schema.json")


@pytest.mark.unit
def test_camera_yaml_valid():
    _validate_yaml("configs/camera.yaml", "configs/schemas/camera.schema.json")


@pytest.mark.unit
def test_slam_toolbox_yaml_valid():
    _validate_yaml("configs/slam_toolbox.yaml", "configs/schemas/slam_toolbox.schema.json")


@pytest.mark.unit
def test_ros2_control_yaml_valid():
    _validate_yaml("configs/ros2_control.yaml", "configs/schemas/ros2_control.schema.json")


@pytest.mark.unit
def test_nav2_yaml_valid():
    _validate_yaml("configs/nav2/nav2_params.yaml", "configs/schemas/nav2.schema.json")


@pytest.mark.unit
def test_diagnostics_yaml_valid():
    _validate_yaml("configs/diagnostics.yaml", "configs/schemas/diagnostics.schema.json")


@pytest.mark.unit
def test_controllers_yaml_valid():
    _validate_yaml("configs/controllers.yaml", "configs/schemas/controllers.schema.json")
