"""Installation script for the 'unitree_rl_mjlab' python package."""

from setuptools import find_packages, setup

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    "mjlab==1.2.0",
    "mujoco==3.5.0",  # Match mujoco-warp; newer APIs remove mjENBL_MULTICCD.
    "mujoco-warp==3.5.0",
    "scipy",  # mjlab 1.2.0 imports SciPy but omits it from its dependencies.
    "onnxruntime==1.22.0",  # Match the bundled C++ ONNX Runtime.
]

# Installation operation
setup(
    name="unitree_rl_mjlab",
    packages=find_packages(
        include=(
            "src", "src.*", "deploy", "deploy.configs", "deploy.configs.*",
            "mjlab_wrapper", "mjlab_wrapper.*",
        )
    ),
    version="0.0.1",
    install_requires=INSTALL_REQUIRES,
    extras_require={"export": ["onnx>=1.16"]},
)
