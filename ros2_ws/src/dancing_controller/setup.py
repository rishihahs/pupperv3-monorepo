from setuptools import find_packages, setup
from glob import glob
import os

package_name = "dancing_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/" + package_name, ["package.xml"]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        ("share/" + package_name + "/resources", glob("resources/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pi",
    maintainer_email="nathankau@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dance = dancing_controller.dance:main",
            "ears = dancing_controller.ears:main",
            "button = dancing_controller.button:main",
        ],
    },
)
