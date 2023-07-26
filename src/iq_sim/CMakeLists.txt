cmake_minimum_required(VERSION 3.8)
project(iq_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rcutils REQUIRED)
find_package(mavros REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

install(DIRECTORY
  launch
  models
  scripts
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/algebra.py
  scripts/fly.py
  scripts/fly_2.py
  scripts/hub.py
  scripts/main.py
  scripts/navigation.py
  scripts/py_gnc_functions.py
  scripts/PrintColours.py
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#   scripts/MDS/UAV
#   DESTINATION lib/${PROJECT_NAME}
# )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_package()
