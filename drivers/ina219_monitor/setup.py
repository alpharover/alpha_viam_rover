from setuptools import setup

package_name = "ina219_monitor"

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
    description="INA219 power monitor publisher for ROS 2 (bus voltage and current).",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ina219_monitor = ina219_monitor.node:main",
        ],
    },
)
