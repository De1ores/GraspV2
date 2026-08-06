cmake_minimum_required(VERSION 3.8)
project(aimdk_msgs VERSION 0.9.0)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CommonRequest.msg"
  "msg/CommonResponse.msg"
  "msg/CommonState.msg"
  "msg/CommonTaskResponse.msg"
  "msg/DomainErrorState.msg"
  "msg/HandCommand.msg"
  "msg/HandCommandArray.msg"
  "msg/HandState.msg"
  "msg/HandStateArray.msg"
  "msg/HandTouchSensorData.msg"
  "msg/HandType.msg"
  "msg/JointCommand.msg"
  "msg/JointCommandArray.msg"
  "msg/JointState.msg"
  "msg/JointStateArray.msg"
  "msg/McAction.msg"
  "msg/McActionCommand.msg"
  "msg/McActionInfo.msg"
  "msg/McActionStatus.msg"
  "msg/McControlArea.msg"
  "msg/McMotion.msg"
  "msg/McMotionType.msg"
  "msg/McPresetMotion.msg"
  "msg/MessageHeader.msg"
  "msg/RequestHeader.msg"
  "msg/ResponseHeader.msg"
  "msg/UpperBodyCommandArray.msg"
  "srv/GetMcAction.srv"
  "srv/GetMcMotions.srv"
  "srv/RegisterCustomMotion.srv"
  "srv/SetMcAction.srv"
  "srv/SetMcMotion.srv"
  "srv/SetMcPresetMotion.srv"
  DEPENDENCIES builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
