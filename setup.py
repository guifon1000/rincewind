from setuptools import setup, find_packages

setup(
    name="rincewind",
    version="0.1.0",
    description="Geometric computation library",
    author="Your Name",
    author_email="your.email@example.com",
    packages=find_packages(),
    install_requires=[
        'numpy',
        'matplotlib',
        'pytest',
        'pytest-cov',
    ],
    python_requires='>=3.6',
    package_data={
        'rincewind': ['assets/*.json'],
    },
    include_package_data=True,
)