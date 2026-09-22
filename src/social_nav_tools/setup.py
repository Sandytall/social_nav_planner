from setuptools import setup

package_name = "social_nav_tools"

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
    maintainer="Sandeep",
    maintainer_email="sandeepnaik89711@gmail.com",
    description="SocialNav tooling: simulated human publisher and RViz markers.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "human_publisher = social_nav_tools.human_publisher:main",
            "human_markers = social_nav_tools.human_markers:main",
        ],
    },
)
