cmake_minimum_required(VERSION 3.5)
project(env_model_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
	"srv/FloatToInt.srv"
	"srv/GetRayTrace.srv"
	"srv/IsMapValid.srv"
	"srv/GetValueAt.srv"
	"srv/GetValueAtPos.srv"	
	"srv/IntToFloat.srv"
	"srv/GetMap.srv"

	DEPENDENCIES builtin_interfaces nav_msgs std_msgs
)


ament_export_dependencies(rosidl_default_runtime)

ament_package()


