from setuptools import setup

package_name = "alpha_viam_bringup"

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
