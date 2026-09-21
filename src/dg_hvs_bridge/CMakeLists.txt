cmake_minimum_required(VERSION 3.16)
project(dg_hvs_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
set(HV_TOOLKIT_ROOT "$ENV{HV_TOOLKIT_ROOT}" CACHE PATH
    "Root of shimetapi_hybrid_vision_toolkit on the RDK X5")

# HVS is an optional RDK-only component.  The NVIDIA workspace normally does
# not contain the vendor Toolkit, so do not make that absence a package
# configuration error.  A complete Toolkit tree enables the real bridge.
set(DG_HVS_BUILD_BRIDGE OFF)
set(DG_HVS_TOOLKIT_MISSING "")
if(NOT HV_TOOLKIT_ROOT)
  set(DG_HVS_TOOLKIT_MISSING "HV_TOOLKIT_ROOT is not set")
else()
  set(HV_INCLUDE_DIR "${HV_TOOLKIT_ROOT}/include")
  set(HV_LIB_DIR "${HV_TOOLKIT_ROOT}/lib/x5")
  set(_required_hv_headers
    shimetapi/codec/mipi_raw8_codec.h
    shimetapi/core/event_cd.h
    shimetapi/core/frame.h
    shimetapi/hv/camera.h
    shimetapi/hv/device_config.h
    shimetapi/io/event_reader.h
    shimetapi/io/event_writer.h)
  foreach(header IN LISTS _required_hv_headers)
    if(NOT EXISTS "${HV_INCLUDE_DIR}/${header}")
      list(APPEND DG_HVS_TOOLKIT_MISSING "${HV_INCLUDE_DIR}/${header}")
    endif()
  endforeach()
  foreach(component core codec io hv)
    set(versioned_library "${HV_LIB_DIR}/libshimetapi_${component}.so.2.0.0")
    if(NOT EXISTS "${versioned_library}")
      list(APPEND DG_HVS_TOOLKIT_MISSING "${versioned_library}")
    endif()
  endforeach()
  if(NOT DG_HVS_TOOLKIT_MISSING)
    set(DG_HVS_BUILD_BRIDGE ON)
  endif()
endif()

if(DG_HVS_BUILD_BRIDGE)
  find_package(rclcpp REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(diagnostic_msgs REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs highgui)

  foreach(component core codec io hv)
    set(versioned_library "${HV_LIB_DIR}/libshimetapi_${component}.so.2.0.0")
    add_library(shimetapi_${component} SHARED IMPORTED)
    set_target_properties(shimetapi_${component} PROPERTIES
      IMPORTED_LOCATION "${versioned_library}"
      INTERFACE_INCLUDE_DIRECTORIES "${HV_INCLUDE_DIR}")
  endforeach()

  add_executable(dg_hvs_bridge_node src/dg_hvs_bridge_node.cpp)
  target_include_directories(dg_hvs_bridge_node PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include)
  target_link_libraries(dg_hvs_bridge_node
    shimetapi_hv shimetapi_io shimetapi_codec shimetapi_core ${OpenCV_LIBS})
  target_link_options(dg_hvs_bridge_node PRIVATE -Wl,--allow-shlib-undefined)
  ament_target_dependencies(dg_hvs_bridge_node rclcpp sensor_msgs diagnostic_msgs)
else()
  message(WARNING
    "dg_hvs_bridge: vendor Toolkit not available; skipping dg_hvs_bridge_node "
    "(RDK only). Missing: ${DG_HVS_TOOLKIT_MISSING}. "
    "Set HV_TOOLKIT_ROOT to a complete shimetapi_hybrid_vision_toolkit tree "
    "when building on RDK X5.")
endif()

# The vendor HBN path is enabled only on an RDK image containing the official
# HVS sample tree. It is deliberately separate from the Jetson robot workspace.
set(DG_HVS_VENDOR_SAMPLE_ROOT
  "/app/multimedia_samples/sample_apx003cc/sample_hvs/dual_vc_vin_src"
  CACHE PATH "Official RDK HVS dual_vc_vin sample directory")
set(DG_HVS_MULTIMEDIA_ROOT "")
if(EXISTS "${DG_HVS_VENDOR_SAMPLE_ROOT}/dual_vc_vin.c")
  get_filename_component(DG_HVS_MULTIMEDIA_ROOT
    "${DG_HVS_VENDOR_SAMPLE_ROOT}/../../.." ABSOLUTE)
endif()

if(DG_HVS_BUILD_BRIDGE AND
   EXISTS "${DG_HVS_VENDOR_SAMPLE_ROOT}/dual_vc_vin.c" AND
   EXISTS "${DG_HVS_MULTIMEDIA_ROOT}/vp_sensors/vp_sensors.c")
  message(STATUS "Enabling official RDK HBN backend from ${DG_HVS_VENDOR_SAMPLE_ROOT}")
  file(GLOB_RECURSE DG_HVS_SENSOR_SOURCES CONFIGURE_DEPENDS
    "${DG_HVS_MULTIMEDIA_ROOT}/vp_sensors/*.c")

  add_library(dg_hvs_hbn_vendor OBJECT
    src/dg_hvs_hbn_backend.c
    "${DG_HVS_VENDOR_SAMPLE_ROOT}/src/hvs_config.c"
    "${DG_HVS_MULTIMEDIA_ROOT}/utils/common_utils.c"
    ${DG_HVS_SENSOR_SOURCES})
  target_compile_definitions(dg_hvs_hbn_vendor PRIVATE
    DG_HVS_DUAL_VC_VIN_SOURCE="${DG_HVS_VENDOR_SAMPLE_ROOT}/dual_vc_vin.c")
  target_compile_options(dg_hvs_hbn_vendor PRIVATE -Wno-unused-function -Wno-unused-parameter)
  target_include_directories(dg_hvs_hbn_vendor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    "${DG_HVS_VENDOR_SAMPLE_ROOT}/include"
    "${DG_HVS_MULTIMEDIA_ROOT}/utils"
    "${DG_HVS_MULTIMEDIA_ROOT}/vp_sensors"
    /usr/include
    /usr/hobot/include)

  add_library(dg_hvs_hbn_display OBJECT
    "${DG_HVS_VENDOR_SAMPLE_ROOT}/src/dual_vc_vin_display.cpp")
  target_compile_options(dg_hvs_hbn_display PRIVATE -Wno-unused-parameter)
  target_include_directories(dg_hvs_hbn_display PRIVATE
    "${DG_HVS_VENDOR_SAMPLE_ROOT}/include"
    "${DG_HVS_MULTIMEDIA_ROOT}/utils"
    "${DG_HVS_MULTIMEDIA_ROOT}/vp_sensors"
    /usr/include
    /usr/hobot/include
    ${OpenCV_INCLUDE_DIRS})

  target_sources(dg_hvs_bridge_node PRIVATE
    $<TARGET_OBJECTS:dg_hvs_hbn_vendor>
    $<TARGET_OBJECTS:dg_hvs_hbn_display>)
  target_compile_definitions(dg_hvs_bridge_node PRIVATE DG_HVS_HAS_HBN_BACKEND=1)
  target_include_directories(dg_hvs_bridge_node PRIVATE
    "${DG_HVS_VENDOR_SAMPLE_ROOT}/include"
    "${DG_HVS_MULTIMEDIA_ROOT}/utils"
    "${DG_HVS_MULTIMEDIA_ROOT}/vp_sensors"
    /usr/include
    /usr/hobot/include)
  target_link_directories(dg_hvs_bridge_node PRIVATE
    /usr/hobot/lib
    "${DG_HVS_VENDOR_SAMPLE_ROOT}/lib")
  target_link_libraries(dg_hvs_bridge_node
    cam vpf hbmem gdcbin cjson alog dl pthread evs_raw_display)
else()
  if(DG_HVS_BUILD_BRIDGE)
    message(STATUS "Official RDK HBN sample tree not found; using Toolkit backend only")
  else()
    message(STATUS "HVS bridge target is disabled because the vendor Toolkit is unavailable")
  endif()
endif()

if(DG_HVS_BUILD_BRIDGE)
  set_target_properties(dg_hvs_bridge_node PROPERTIES
    BUILD_RPATH "${HV_LIB_DIR}"
    INSTALL_RPATH "${HV_LIB_DIR};$ORIGIN"
    INSTALL_RPATH_USE_LINK_PATH TRUE)
  install(TARGETS dg_hvs_bridge_node DESTINATION lib/${PROJECT_NAME})
endif()
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES include/dg_hvs_hbn_backend.h DESTINATION include/${PROJECT_NAME})
install(PROGRAMS
  scripts/run_hvs_capture_session.sh
  scripts/run_three_view_capture.sh
  scripts/show_fused_image.py
  scripts/run_hvs_display.sh
  DESTINATION lib/${PROJECT_NAME})

ament_package()
