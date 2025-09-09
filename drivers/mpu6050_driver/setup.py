from setuptools import setup

package_name = "mpu6050_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="alpha_viam",
    maintainer_email="alpha@viam.local",
    description="MPU-6050 IMU publisher for ROS 2 (sensor_msgs/Imu).",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mpu6050_node = mpu6050_driver.node:main",
        ],
    },
)
