import os
from glob import glob
from setuptools import setup

package_name = "ahpu_odom_nav"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alex",
    maintainer_email="198358184+aixuexidexiaoshuai@users.noreply.github.com",
    description="惯导(纯陀螺)+轮式里程计的 teach-and-repeat 定点巡航（不依赖 GPS/SLAM/地图/Nav2）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "icm20948_driver = ahpu_odom_nav.icm20948_driver:main",
            "waypoint_capture = ahpu_odom_nav.waypoint_capture:main",
            "smooth_path = ahpu_odom_nav.smooth_path:main",
            "odom_waypoint_follower = ahpu_odom_nav.odom_waypoint_follower:main",
            "odom_monitor = ahpu_odom_nav.odom_monitor:main",
        ],
    },
)
