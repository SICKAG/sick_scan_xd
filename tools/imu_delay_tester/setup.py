# 
# Setup Skeleton taken from here: https://github.com/maet3608/minimal-setup-py/blob/master/setup.py
#
from setuptools import setup, find_packages

setup(
    name='imu_delay_tester',
    version='1.0.0',
    url='https://github.com/SICKAG/sick_scan_xd/tools/imu_delay_test',
    author='Michael Lehning',
    author_email='michael.lehning@lehning.de',
    description='Test for multiScan IMU',
    packages=find_packages(),    
	#
	# "requests" is needed for URL handling.
    install_requires=['numpy',
                      'matplotlib',
                      'scipy',
                      'statsmodels'],
)