try:
    # Try using ez_setup to install setuptools if not already installed.
    from ez_setup import use_setuptools
    use_setuptools()
except ImportError:
    # Ignore import error and assume Python 3 which already has setuptools.
    pass

from setuptools import setup, find_packages

classifiers = ['Development Status :: 4 - Beta',
               'Operating System :: POSIX :: Linux',
               'License :: OSI Approved :: MIT License',
               'Intended Audience :: Developers',
               'Programming Language :: Python :: 2.7',
               'Programming Language :: Python :: 3',
               'Topic :: Software Development',
               'Topic :: System :: Hardware']

setup(name              = 'i2cpwmcontroller',
      version           = '1.0.1',
      author            = 'Bradan Lane Studio',
      author_email      = 'info@bradanlane.com',
      description       = 'Python code to use the PCA9685 I2C PWM servo controller with a Raspberry Pi.',
      license           = 'MIT',
      classifiers       = classifiers,
      url               = 'https://gitlab.com/bradanlane/py-locoro/',
      dependency_links  = ['https://github.com/adafruit/Adafruit_Python_PCA9685/tarball/master#egg=Adafruit_PCA9685-1.0.1'],
      install_requires  = ['Adafruit-PCA9685>=1.0.1'],
      packages          = find_packages())

