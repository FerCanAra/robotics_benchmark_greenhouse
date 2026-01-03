from setuptools import setup
from glob import glob

package_name = "benchmark_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name, package_name + ".categories", package_name + ".plotter"],
    data_files=[
        # Resource index
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        # package.xml
        ("share/" + package_name, ["package.xml"]),
        # Launch files
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        # Definitions
        ("share/" + package_name + "/definitions/robots", glob("definitions/robots/*")),
        ("share/" + package_name + "/definitions/world", glob("definitions/world/*")),
        (
            "share/" + package_name + "/definitions/sensors",
            glob("definitions/sensors/*"),
        ),
        # Maps
        ("share/" + package_name + "/maps", glob("maps/*")),
        # Params
        ("share/" + package_name + "/control_params", glob("control_params/*")),
        # Configuration
        ("share/" + package_name + "/configuration", glob("configuration/*")),
        # RVIZ
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Fernando Cañadas Aránega",
    maintainer_email="fernando.ca@ual.es",
    description="Benchmark bringup for greenhouse robotics.",
    license="BSD",
    entry_points={
        "console_scripts": [
            "velocity_cat1    = benchmark_bringup.categories.categorie_1:main",
            "route_cat2       = benchmark_bringup.categories.categorie_2:main",
            "route_cat3       = benchmark_bringup.categories.categorie_3:main",
            "plotter_c1_benchmark = benchmark_bringup.plotter.plotter_c1:main",
            "plotter_c2_benchmark = benchmark_bringup.plotter.plotter_c2:main",
            "plotter_c3_benchmark = benchmark_bringup.plotter.plotter_c3:main",
            "odom_mvsim2teb = benchmark_bringup.odom_mvsim2TEB:main",
        ],
    },
)
