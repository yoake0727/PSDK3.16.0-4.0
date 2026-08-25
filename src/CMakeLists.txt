cmake_minimum_required(VERSION 3.5)

project(psdk_payload_app CXX)

set(CMAKE_C_FLAGS "-pthread -std=gnu99")
set(CMAKE_CXX_FLAGS "-std=c++11 -pthread")
set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")
set(CMAKE_EXE_LINKER_FLAGS "-pthread")

add_definitions(-D_GNU_SOURCE)

if (BUILD_TEST_CASES_ON MATCHES TRUE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fprofile-arcs -ftest-coverage -Wno-deprecated-declarations")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fprofile-arcs -ftest-coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -fprofile-arcs -ftest-coverage -lgcov")
endif ()

include_directories(module_sample)
include_directories(common)
include_directories(application)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../psdk_lib/include)

file(GLOB_RECURSE MODULE_SAMPLE_SRC
    module_sample/liveview/*.c*
    module_sample/camera_manager/*.c*
    module_sample/hms_manager/*.c*
    module_sample/perception/*.c*
    module_sample/gimbal/*.c*
    module_sample/flight_controller/*.c*
    module_sample/widget_manager/*.c*
    module_sample/positioning/*.c*
    )
file(GLOB_RECURSE MODULE_COMMON_SRC
    common/*.c*
    )
file(GLOB_RECURSE MODULE_HAL_SRC hal/*.c*)
file(GLOB_RECURSE MODULE_APP_SRC application/*.c*)
list(REMOVE_ITEM MODULE_APP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/application/main2.cpp")

set(TOOLCHAIN_NAME aarch64-linux-gnu-gcc)
add_definitions(-DPLATFORM_ARCH_AARCH64=1)
add_definitions(-DSYSTEM_ARCH_LINUX=1)

get_filename_component(REPOSITORY_ROOT
        "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

set(STATIC_DEPS_ROOT "${REPOSITORY_ROOT}/third_party/aarch64")
set(STATIC_DEPS_INCLUDE_DIR "${STATIC_DEPS_ROOT}/include")
set(STATIC_DEPS_LIBRARY_DIR "${STATIC_DEPS_ROOT}/lib")
set(MOSQUITTO_STATIC_LIBRARY
        "${STATIC_DEPS_LIBRARY_DIR}/libmosquitto.a")
set(FFMPEG_STATIC_LIBRARIES
        "${STATIC_DEPS_LIBRARY_DIR}/libavformat.a"
        "${STATIC_DEPS_LIBRARY_DIR}/libavcodec.a"
        "${STATIC_DEPS_LIBRARY_DIR}/libswscale.a"
        "${STATIC_DEPS_LIBRARY_DIR}/libavutil.a")

set(REQUIRED_STATIC_DEPENDENCIES
        "${STATIC_DEPS_INCLUDE_DIR}/mosquitto.h"
        "${STATIC_DEPS_INCLUDE_DIR}/libavformat/avformat.h"
        "${STATIC_DEPS_INCLUDE_DIR}/libavcodec/avcodec.h"
        "${STATIC_DEPS_INCLUDE_DIR}/libavutil/avutil.h"
        "${STATIC_DEPS_INCLUDE_DIR}/libswscale/swscale.h"
        "${MOSQUITTO_STATIC_LIBRARY}"
        ${FFMPEG_STATIC_LIBRARIES})
foreach(REQUIRED_STATIC_DEPENDENCY ${REQUIRED_STATIC_DEPENDENCIES})
    if (NOT EXISTS "${REQUIRED_STATIC_DEPENDENCY}")
        message(FATAL_ERROR
                "Required Manifold 3 static dependency not found: ${REQUIRED_STATIC_DEPENDENCY}. "
                "Run tools/build_manifold3_static_deps.sh on Manifold 3 first.")
    endif ()
endforeach()

link_directories(${CMAKE_CURRENT_LIST_DIR}/../psdk_lib/lib/${TOOLCHAIN_NAME})
link_libraries(${CMAKE_CURRENT_LIST_DIR}/../psdk_lib/lib/${TOOLCHAIN_NAME}/libpayloadsdk.a -lstdc++)

add_executable(${PROJECT_NAME}
        ${MODULE_APP_SRC}
        ${MODULE_SAMPLE_SRC}
        ${MODULE_COMMON_SRC}
        ${MODULE_HAL_SRC})

include_directories(${CMAKE_CURRENT_LIST_DIR}/../psdk_lib/include)

# Try to see if user has OpenCV installed
find_package(OpenCV QUIET)
if (OpenCV_FOUND)
    message("\n${PROJECT_NAME}...")
    message(STATUS "Found OpenCV installed in the system, will use it to display image in AdvancedSensing APIs")
    message(STATUS " - Includes: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS " - Libraries: ${OpenCV_LIBRARIES}")
    add_definitions(-DOPEN_CV_INSTALLED)
    target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
else ()
    message(STATUS "Did not find OpenCV in the system, image data is inside RecvContainer as raw data")
endif ()

message(STATUS "Using static FFmpeg and Mosquitto from: ${STATIC_DEPS_ROOT}")
target_include_directories(${PROJECT_NAME} PRIVATE "${STATIC_DEPS_INCLUDE_DIR}")
add_definitions(-DFFMPEG_INSTALLED)

if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
endif()

if (NOT EXECUTABLE_OUTPUT_PATH)
    set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
endif()

target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

if (OpenCV_FOUND)
    target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
endif()

target_link_libraries(${PROJECT_NAME}
        "-Wl,--start-group"
        ${FFMPEG_STATIC_LIBRARIES}
        ${MOSQUITTO_STATIC_LIBRARY}
        "-Wl,--end-group"
        pthread
        m
        dl)

set(JSON_FILE ${CMAKE_CURRENT_SOURCE_DIR}/app_json/app.json)
set(SCRIPT_FILE ${CMAKE_CURRENT_SOURCE_DIR}/../tools/build_dpk/build_dpk.sh)

set(OUTPUT_PACKAGE ${CMAKE_BINARY_DIR}/dpk)

add_custom_command(TARGET ${PROJECT_NAME}
    POST_BUILD
    COMMAND bash ${SCRIPT_FILE} -i ${JSON_FILE} -o ${OUTPUT_PACKAGE}
    DEPENDS ${JSON_FILE}
    COMMENT "Running build_dpk.sh, generating .dpk app package from app.json file"
)