from setuptools import setup

package_name = "wifi_monitor"

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
    description="Wi‑Fi link health publisher (RSSI dBm and link_ok) for ROS 2.",
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wifi_monitor = wifi_monitor.node:main",
        ],
    },
)
