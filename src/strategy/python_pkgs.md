# Build python ros pkg guide
------------------------------------
* Installing scripts and exporting modules
```
| - workspace/
| - - src/
| - - - my_pkg/
| - - - - *setup.py*
| - - - - *CMakeLists.txt*
| - - - - bin/
| - - - - - *wrs_strategy*  (execution file)
| - - - - src/
| - - - - - wrs_strategy/
| - - - - - - *__init__.py*
| - - - - - - *wrs_strategy.py*
```

* Build CMakeLists.txt
```
cmake_minimum_required(VERSION 2.8.3)
project(strategy)

find_package(catkin REQUIRED COMPONENTS message_generation rospy)

# Uncomment if the package has a setup.py
catkin_python_setup()


# Generate added messages and services with any dependencies
generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime 
)

# Install the script to the right location
catkin_install_python(PROGRAMS bin/wheel_strategy
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
```

* Make the file executable:     
`chmod u+x bin/wrs_strategy`

* Creat `setup.py`         
```
## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['wrs_strategy'],
    package_dir={'': 'src'},
)

setup(**setup_args)
```