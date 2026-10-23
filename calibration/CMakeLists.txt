cmake_minimum_required(VERSION 3.14)
project(awakening_calibration LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
get_filename_component(AWAKENING_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
list(APPEND CMAKE_MODULE_PATH "${AWAKENING_ROOT}/cmake")
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
find_package(TBB REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)
find_package(HikSDK QUIET)
find_package(MvSDK QUIET)
find_package(DahengSDK QUIET)
# The common headers require param_deliver.h; keep the repository root in it.
set(saved_source_dir "${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_CURRENT_SOURCE_DIR "${AWAKENING_ROOT}")
configure_file("${AWAKENING_ROOT}/src/param_deliver.h.in" "${CMAKE_CURRENT_BINARY_DIR}/param_deliver.h" @ONLY)
set(CMAKE_CURRENT_SOURCE_DIR "${saved_source_dir}")
add_executable(calibration_capture capture.cpp
    "${AWAKENING_ROOT}/src/utils/drivers/camera_factory.cpp"
    "${AWAKENING_ROOT}/src/utils/drivers/hik_camera.cpp"
    "${AWAKENING_ROOT}/src/utils/drivers/mv_camera.cpp"
    "${AWAKENING_ROOT}/src/utils/drivers/daheng_camera.cpp")
target_include_directories(calibration_capture PRIVATE "${AWAKENING_ROOT}/src"
    "${AWAKENING_ROOT}/3rdparty" "${CMAKE_CURRENT_BINARY_DIR}" ${OpenCV_INCLUDE_DIRS})
target_link_libraries(calibration_capture PRIVATE ${OpenCV_LIBS} Eigen3::Eigen yaml-cpp
    fmt::fmt spdlog::spdlog TBB::tbb Boost::system Ceres::ceres Threads::Threads)
foreach(sdk IN ITEMS Hik Mv Daheng)
    if(${sdk}SDK_FOUND)
        target_compile_definitions(calibration_capture PRIVATE USE_${sdk}SDK)
        target_link_libraries(calibration_capture PRIVATE ${sdk}SDK::${sdk}SDK)
    endif()
endforeach()
