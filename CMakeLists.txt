cmake_minimum_required(VERSION 3.16)
project(vehicle LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

set(DS_ROOT "/opt/nvidia/deepstream/deepstream" CACHE PATH "DeepStream install root")
option(BUILD_TESTS "Build C++ business unit tests" ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 glib-2.0 gobject-2.0)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
pkg_check_modules(CURL REQUIRED libcurl)
find_library(YAMLCPP_LIB NAMES yaml-cpp REQUIRED)

add_library(vehicle_business STATIC
  src/common/logging.cpp
  src/common/types.cpp
  src/common/config.cpp
  src/utils/string_utils.cpp
  src/utils/geometry.cpp
  src/utils/time_utils.cpp
  src/utils/latency.cpp
  src/utils/jpeg_draw.cpp
  src/business/plate/rules.cpp
  src/business/plate/track.cpp
  src/business/plate/event.cpp
  src/business/plate/plate_recognizer.cpp
  src/business/violation/config_store.cpp
  src/probes/char_assembler.cpp
)
target_include_directories(vehicle_business PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${JSONCPP_INCLUDE_DIRS}
)
find_package(JPEG REQUIRED)
target_include_directories(vehicle_business PUBLIC ${JPEG_INCLUDE_DIRS})
target_link_libraries(vehicle_business PUBLIC ${JSONCPP_LIBRARIES} ${YAMLCPP_LIB} ${JPEG_LIBRARIES} pthread)
target_link_directories(vehicle_business PUBLIC ${JSONCPP_LIBRARY_DIRS})

# ---------------------------------------------------------------------------
# vehicle — ứng dụng DeepStream.
# ---------------------------------------------------------------------------
add_executable(vehicle
  main.cpp
  src/communication/mqtt_client.cpp
  src/communication/vms_client.cpp
  src/communication/http_uploader.cpp
  src/communication/event_publisher.cpp
  src/probes/plate_probe.cpp
  src/pipeline/pipeline.cpp
  src/pipeline/watchers.cpp
  src/utils/plate_snapshot_warp.cpp
)
target_include_directories(vehicle PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${DS_ROOT}/sources/includes
  ${GST_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

find_library(CUDA_DRIVER_LIB NAMES cuda
  PATHS /usr/local/cuda/lib64/stubs /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH)
if(NOT CUDA_DRIVER_LIB)
  message(WARNING "Không tìm thấy libcuda — link có thể lỗi undefined reference cu*")
endif()
find_library(CUDART_LIB NAMES cudart
  PATHS /usr/local/cuda/lib64 /usr/lib/x86_64-linux-gnu
  NO_DEFAULT_PATH)
find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
  PATHS /usr/local/cuda/include /usr/include)

target_link_directories(vehicle PRIVATE ${DS_ROOT}/lib ${GST_LIBRARY_DIRS} ${CURL_LIBRARY_DIRS})
if(CUDA_INCLUDE_DIR)
  target_include_directories(vehicle PRIVATE ${CUDA_INCLUDE_DIR})
endif()
target_link_libraries(vehicle PRIVATE
  vehicle_business
  ${GST_LIBRARIES}
  ${CURL_LIBRARIES}
  nvdsgst_meta
  nvds_meta
  nvds_batch_jpegenc
  nvbufsurface
  nvbufsurftransform
  ${CUDA_DRIVER_LIB}
  ${CUDART_LIB}
  dl
)
set_target_properties(vehicle PROPERTIES
  BUILD_RPATH "${DS_ROOT}/lib"
  INSTALL_RPATH "${DS_ROOT}/lib"
)


set(LIBS_OUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/libs")

set(CUDA_VER "" CACHE STRING "CUDA version for DeepStream custom libs")
if(NOT CUDA_VER)
  execute_process(
    COMMAND bash -c "readlink -f /usr/local/cuda | sed -n 's|.*/cuda-\\([0-9][0-9]*\\.[0-9][0-9]*\\).*|\\1|p'"
    OUTPUT_VARIABLE CUDA_VER
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
  )
endif()
if(NOT CUDA_VER)
  message(FATAL_ERROR "Không xác định được CUDA_VER — truyền -DCUDA_VER=12.8 khi cmake")
endif()
message(STATUS "DeepStream custom libs: CUDA_VER=${CUDA_VER}")

# --- YOLO bbox parser (PGIE vehicle / SGIE digit) ---
set(YOLO_IMPL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources/ds/nvdsinfer_custom_impl_Yolo")
set(YOLO_IMPL_SO "${YOLO_IMPL_DIR}/libnvdsinfer_custom_impl_Yolo.so")
set(YOLO_OUT_SO "${LIBS_OUT_DIR}/libnvdsinfer_custom_impl_Yolo.so")
file(GLOB YOLO_IMPL_SOURCES
  "${YOLO_IMPL_DIR}/*.cpp"
  "${YOLO_IMPL_DIR}/*.cu"
  "${YOLO_IMPL_DIR}/*.h"
  "${YOLO_IMPL_DIR}/layers/*.cpp"
  "${YOLO_IMPL_DIR}/layers/*.h"
)
add_custom_command(
  OUTPUT "${YOLO_OUT_SO}"
  COMMAND ${CMAKE_COMMAND} -E env CUDA_VER=${CUDA_VER} make -C "${YOLO_IMPL_DIR}" -j
  COMMAND ${CMAKE_COMMAND} -E make_directory "${LIBS_OUT_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${YOLO_IMPL_SO}" "${YOLO_OUT_SO}"
  WORKING_DIRECTORY "${YOLO_IMPL_DIR}"
  DEPENDS ${YOLO_IMPL_SOURCES} "${YOLO_IMPL_DIR}/Makefile"
  COMMENT "Building libnvdsinfer_custom_impl_Yolo.so (CUDA_VER=${CUDA_VER})"
  VERBATIM
)
add_custom_target(nvdsinfer_custom_impl_Yolo ALL DEPENDS "${YOLO_OUT_SO}")

# --- YOLO-Pose parser (SGIE1 plate keypoints) ---
set(POSE_IMPL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources/ds/nvdsinfer_custom_impl_Yolo_pose")
set(POSE_IMPL_SO "${POSE_IMPL_DIR}/libnvdsinfer_custom_impl_Yolo_pose.so")
set(POSE_OUT_SO "${LIBS_OUT_DIR}/libnvdsinfer_custom_impl_Yolo_pose.so")
file(GLOB POSE_IMPL_SOURCES "${POSE_IMPL_DIR}/*.cpp" "${POSE_IMPL_DIR}/*.h")
add_custom_command(
  OUTPUT "${POSE_OUT_SO}"
  COMMAND ${CMAKE_COMMAND} -E env CUDA_VER=${CUDA_VER} make -C "${POSE_IMPL_DIR}" -j
  COMMAND ${CMAKE_COMMAND} -E make_directory "${LIBS_OUT_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${POSE_IMPL_SO}" "${POSE_OUT_SO}"
  WORKING_DIRECTORY "${POSE_IMPL_DIR}"
  DEPENDS ${POSE_IMPL_SOURCES} "${POSE_IMPL_DIR}/Makefile"
  COMMENT "Building libnvdsinfer_custom_impl_Yolo_pose.so (CUDA_VER=${CUDA_VER})"
  VERBATIM
)
add_custom_target(nvdsinfer_custom_impl_Yolo_pose ALL DEPENDS "${POSE_OUT_SO}")

# --- Warp perspective preprocess (plate flatten trước digit) ---
set(WARP_IMPL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/resources/ds/nvdspreprocess_custom_warp_perspective")
set(WARP_IMPL_SO "${WARP_IMPL_DIR}/libnvdspreprocess_custom_warp_perspective.so")
set(WARP_OUT_SO "${LIBS_OUT_DIR}/libnvdspreprocess_custom_warp_perspective.so")
file(GLOB WARP_IMPL_SOURCES
  "${WARP_IMPL_DIR}/*.cpp"
  "${WARP_IMPL_DIR}/*.cu"
  "${WARP_IMPL_DIR}/*.h"
)
add_custom_command(
  OUTPUT "${WARP_OUT_SO}"
  COMMAND ${CMAKE_COMMAND} -E env CUDA_VER=${CUDA_VER} make -C "${WARP_IMPL_DIR}" -j
  COMMAND ${CMAKE_COMMAND} -E make_directory "${LIBS_OUT_DIR}"
  COMMAND ${CMAKE_COMMAND} -E copy_if_different "${WARP_IMPL_SO}" "${WARP_OUT_SO}"
  WORKING_DIRECTORY "${WARP_IMPL_DIR}"
  DEPENDS ${WARP_IMPL_SOURCES} "${WARP_IMPL_DIR}/Makefile"
  COMMENT "Building libnvdspreprocess_custom_warp_perspective.so (CUDA_VER=${CUDA_VER})"
  VERBATIM
)
add_custom_target(nvdspreprocess_custom_warp_perspective ALL DEPENDS "${WARP_OUT_SO}")

if(BUILD_TESTS)
  add_executable(vehicle_business_tests tests/business_cpp/test_business.cpp)
  target_link_libraries(vehicle_business_tests PRIVATE vehicle_business)
  enable_testing()
  add_test(NAME business COMMAND vehicle_business_tests)
endif()
