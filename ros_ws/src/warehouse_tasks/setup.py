from glob import glob

from setuptools import setup

PKG = "warehouse_tasks"

setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PKG}"]),
        (f"share/{PKG}", ["package.xml"]),
        (f"share/{PKG}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="MoveBox task manager (action server + pick/place state machine).",
    license="MIT",
    entry_points={"console_scripts": [
        f"task_manager = {PKG}.task_manager:main",
        f"fleet_manager = {PKG}.fleet_manager:main",
    ]},
)
