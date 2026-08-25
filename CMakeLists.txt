cmake_minimum_required(VERSION 2.8)

if (NOT USE_SYSTEM_ARCH)
    # select use platform 'LINUX' or 'RTOS' here, reset cache and reload cmake project
    set(USE_SYSTEM_ARCH LINUX)
endif ()

set(LOCAL_DEPS_DIR $ENV{HOME}/local)
set(LOCAL_INCLUDE_DIR ${LOCAL_DEPS_DIR}/include)
set(LOCAL_LIB_DIR ${LOCAL_DEPS_DIR}/lib)

project(entry)

# Disable in-source builds to prevent source tree corruption.
if (" ${CMAKE_SOURCE_DIR}" STREQUAL " ${CMAKE_BINARY_DIR}")
    message(FATAL_ERROR "FATAL: In-source builds are not allowed.
       You should create a separate directory for build files.")
endif ()

if (USE_SYSTEM_ARCH MATCHES RTOS)
    cmake_minimum_required(VERSION 3.15)
    set(CMAKE_C_COMPILER arm-none-eabi-gcc)
    set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
    set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
    set(CMAKE_AR arm-none-eabi-ar)
    set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
    set(CMAKE_OBJDUMP arm-none-eabi-objdump)
    set(SIZE arm-none-eabi-size)
    set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
    
    add_definitions(-DSYSTEM_ARCH_RTOS)
    add_subdirectory(samples/sample_c/platform/rtos_freertos/stm32f4_discovery/project/armgcc)
    
elseif (USE_SYSTEM_ARCH MATCHES LINUX)
    add_definitions(-DSYSTEM_ARCH_LINUX)
    
    # ======================== 修改开始 ========================
    # 原路径：${LOCAL_INCLUDE_DIR}  →  新路径：${CMAKE_CURRENT_SOURCE_DIR}/psdk_lib/include
    include_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/psdk_lib/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src/application
        ${CMAKE_CURRENT_SOURCE_DIR}/src/common
        ${CMAKE_CURRENT_SOURCE_DIR}/src/module_sample
        ${CMAKE_CURRENT_SOURCE_DIR}/src/hal
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/aarch64/include
    )
    
    link_directories(
        ${CMAKE_CURRENT_SOURCE_DIR}/psdk_lib/lib/aarch64-linux-gnu-gcc
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/aarch64/lib
    )
    # ======================== 修改结束 ========================

    # ======================== 修改开始 ========================
    # 原路径：${LOCAL_LIB_DIR}/libcjson.a  →  已不需要，cjson 不在依赖中
    # 但保留检查逻辑，改为检查必要的库
    set(PSDK_LIB ${CMAKE_CURRENT_SOURCE_DIR}/psdk_lib/lib/aarch64-linux-gnu-gcc/libpayloadsdk.a)
    set(MOSQUITTO_STATIC_LIB ${CMAKE_CURRENT_SOURCE_DIR}/third_party/aarch64/lib/libmosquitto.a)
    
    if (NOT EXISTS ${PSDK_LIB})
        message(FATAL_ERROR "Static library libpayloadsdk.a not found in psdk_lib/lib/aarch64-linux-gnu-gcc")
    endif()
    
    if (NOT EXISTS ${MOSQUITTO_STATIC_LIB})
        message(FATAL_ERROR "Static library libmosquitto.a not found in third_party/aarch64/lib")
    endif()
    # ======================== 修改结束 ========================
    
    message(STATUS "Using PSDK lib: ${PSDK_LIB}")
    message(STATUS "Using static libmosquitto: ${MOSQUITTO_STATIC_LIB}")

    set(EXTRA_SYS_LIBS pthread ssl crypto)

    # ======================== 修改开始 ========================
    # 原路径：add_subdirectory(samples/sample_c++/platform/linux/manifold3)
    # 新路径：add_subdirectory(src)  →  直接添加 src 子目录
    add_subdirectory(src)
    # ======================== 修改结束 ========================
    
    execute_process(COMMAND uname -m OUTPUT_VARIABLE DEVICE_SYSTEM_ID)
    if (DEVICE_SYSTEM_ID MATCHES x86_64)
        set(LIBRARY_PATH psdk_lib/lib/x86_64-linux-gnu-gcc)
    elseif (DEVICE_SYSTEM_ID MATCHES aarch64)
        set(LIBRARY_PATH psdk_lib/lib/aarch64-linux-gnu-gcc)
    else ()
        message(FATAL_ERROR "FATAL: Please confirm your platform.")
    endif ()

    install(FILES ${LIBRARY_PATH}/libpayloadsdk.a
            DESTINATION "${CMAKE_INSTALL_PREFIX}/lib"
            )

    install(DIRECTORY psdk_lib/include
            DESTINATION "${CMAKE_INSTALL_PREFIX}"
            )

endif ()

add_custom_target(${PROJECT_NAME} ALL)