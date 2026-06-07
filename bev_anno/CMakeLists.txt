cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON) # eCAL/proto

# "../" --> output directory relative same to running in IDE
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/../)

if (MSVC)
  set(CMAKE_CXX_STANDARD 11)
endif()

project(bev_anno)

if(MSVC)
  # CMake >= 3.15 will erroneously define PROTOBUF_USE_DLLS otherwise
  set (Protobuf_USE_STATIC_LIBS TRUE) 
endif()

find_package(eCAL REQUIRED)
find_package(Protobuf REQUIRED)

set(protobuf_files
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/pcl.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/pcl_header.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/pcl_pfield.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/types.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/lane.proto # 2do: remove
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/gsm.proto  # 2do: simplify!
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/LinePrimitive.proto
# added for SceneUpdate
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/SceneEntity.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/SceneUpdate.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/ArrowPrimitive.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/CubePrimitive.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/CylinderPrimitive.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/KeyValuePair.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/ModelPrimitive.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/SpherePrimitive.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/TextPrimitive.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/TriangleListPrimitive.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/SceneEntityDeletion.proto
# added for SceneUpdate
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/Color.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/Pose.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/Point3.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/Quaternion.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/Vector3.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/PointCloud.proto
  ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/foxglove/PackedElementField.proto
)

include_directories(.
src
include/features
include/math
include/pointcloud
include/util
)

# legacy ->
link_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../lib)

# add WIN32 to remove console window ->
add_executable (${CMAKE_PROJECT_NAME} #WIN32
src/main.cpp
src/main.h
src/app_ecal.hpp
src/app_algowrp.hpp
src/anno_io.h
src/calc_ground_pt.h
# interface
if/hndl_pointcloud.cpp
if/hndl_pointcloud.h
# includes
include/pointcloud/pointcloud.hpp
include/pointcloud/pointcloud_io.hpp
include/pointcloud/pointcloud_synth.hpp
include/math/linmath.h
include/math/mathbase.h
include/util/color.h
include/util/nlohmann/json.hpp
include/util/nlohmann/json_fwd.hpp
include/util/config_parser.h
include/util/config_parser.cpp
)

PROTOBUF_TARGET_CPP(${PROJECT_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/if/proto/ ${protobuf_files})

#add_custom_command(
#        TARGET ${PROJECT_NAME}  POST_BUILD
#        COMMAND ${CMAKE_COMMAND} -E copy
#                ${SDL2_DIR}/../lib/x64/SDL2.dll
#                ${TARGET_FILE_DIR}/SDL2.dll)

target_link_libraries(${CMAKE_PROJECT_NAME}
  eCAL::core protobuf::libprotobuf
)

if (MSVC)
else()
# Linux -->
target_link_libraries(${PROJECT_NAME} ${CMAKE_DL_LIBS})
endif()
