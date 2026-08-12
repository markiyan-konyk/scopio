cmake_minimum_required(VERSION 3.8)
project(scopio_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # ---- Messages (SCOPIO interface contract v2.0 -- treat as frozen) ----
  "msg/StagePosition.msg"
  "msg/StagePoint.msg"
  "msg/CameraState.msg"
  "msg/AwgStatus.msg"
  "msg/TemperatureStatus.msg"
  "msg/Calibration.msg"
  # ---- Services ----
  "srv/StageJog.srv"
  "srv/MoveAbs.srv"
  "srv/SetCameraControls.srv"
  "srv/SetFramerate.srv"
  "srv/WhiteBalance.srv"
  "srv/AwgWrite.srv"
  "srv/AwgQuery.srv"
  "srv/CalibrationSet.srv"
  "srv/InstrumentCall.srv"   # generic driver-method dispatch (galvo)
  # ---- Actions ----
  "action/MoveStagePath.action"
  "action/ScanRegion.action"
  "action/Autofocus.action"
  DEPENDENCIES std_msgs
)

ament_package()
