import setuptools

setuptools.setup(
    name="a1z",
    version="0.0.1",
    description="SDK for the A1Z 6-DOF robotic arm",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
    ],
    python_requires=">=3.10",
    install_requires=[
        "numpy",
        "python-can>=4.0",
        "pin",
        "pyyaml",
    ],
    extras_require={
        # gs_usb userspace backend (macOS/Windows with HHS adapter)
        # libusb-package bundles the libusb binary (no brew/zadig libusb needed)
        "gs_usb": [
            "python-can[gs_usb]",
            "libusb-package",
            "libusb1>=3.1,<4",
        ],
        # PEAK PCAN-USB adapter
        "pcan": [
            "python-can[pcan]",
        ],
    },
)
