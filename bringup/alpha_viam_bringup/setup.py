from setuptools import setup
import os

package_name = "alpha_viam_bringup"

# Use paths relative to this setup.py (colcon/ament requires non-absolute data_files)
_cfg = lambda name: os.path.join("..", "..", "configs", name)
_urdf = lambda name: os.path.join("..", "..", "urdf", name)

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", [
            "alpha_viam_bringup/launch/base_bringup.launch.py",
            "alpha_viam_bringup/launch/drive_min.launch.py",
            "alpha_viam_bringup/launch/drive_direct.launch.py",
            "alpha_viam_bringup/launch/drive_forward.launch.py",
            "alpha_viam_bringup/launch/cm_only.launch.py",
        ]),
        # Install selected configs and URDF into the package share for robust pathing
        ("share/" + package_name + "/configs", [
            _cfg("controllers.yaml"),
            _cfg("diff_drive.params.yaml"),
            _cfg("diff_drive_params.yaml"),
            _cfg("wheels_forward.yaml"),
            _cfg("ekf.yaml"),
            _cfg("diagnostics.yaml"),
            _cfg("network.yaml"),
            _cfg("power.yaml"),
            _cfg("imu.yaml"),
        ]),
        ("share/" + package_name + "/configs/spawner", [
            _cfg("spawner/forward.yaml"),
        ]),
        ("share/" + package_name + "/urdf", [
            _urdf("rover.urdf.xacro"),
        ]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="alpha_viam",
    maintainer_email="alpha@viam.local",
    description="Bring-up launch files for the Alpha Viam Rover (URDF, EKF, Foxglove).",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
