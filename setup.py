try:
    from skbuild import setup
except ImportError:
    raise Exception

setup(
    name="psdf",
    version="0.0.1",
    description="primitive sdf",
    author="Hirokazu Ishida",
    license="MIT",
    install_requires=["numpy"],
    packages=["psdf"],
    package_dir={"": "python"},
    package_data={"psdf": ["__init__.pyi"]},
    cmake_install_dir="python/psdf/",
)
