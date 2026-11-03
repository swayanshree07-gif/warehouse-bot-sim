from glob import glob

from setuptools import setup

PKG = "warehouse_bringup"

setup(
    name=PKG,
    version="0.1.0",
    packages=[PKG],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PKG}"]),
        (f"share/{PKG}", ["package.xml"]),
        (f"share/{PKG}/launch", glob("launch/*.launch.py")),
        (f"share/{PKG}/urdf", glob("urdf/*.urdf")),
        (f"share/{PKG}/config", glob("config/*.yaml")),
        (f"share/{PKG}/maps", glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    description="Launch files and robot description for the Isaac Sim warehouse robot.",
    license="MIT",
)
