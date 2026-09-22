from glob import glob

from setuptools import setup

package_name = "social_nav_rl"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sandeep",
    maintainer_email="sandeepnaik89711@gmail.com",
    description="Experimental RL social navigation policy (observation/reward/safety core "
                "+ optional PPO training).",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            # These need the pip extras (gymnasium, stable_baselines3); they exit with a clear
            # message if those are missing.
            "social-nav-rl-train = social_nav_rl.train:main",
            "social-nav-rl-eval = social_nav_rl.evaluate:main",
        ],
    },
)
