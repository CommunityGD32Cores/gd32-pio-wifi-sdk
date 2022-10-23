
cmake_minimum_required(VERSION 3.15)

add_library(iperf3_api INTERFACE)

target_include_directories(iperf3_api
INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)


if (NOT CONFIG_STATIC_LIB_BUILDED)
    add_library(iperf3 STATIC)
    file(GLOB_RECURSE SRC_IPERF3  ${CMAKE_CURRENT_SOURCE_DIR}/*.c)


    target_sources(iperf3
        PRIVATE
            ${SRC_IPERF3}
            cpu_a_gcc.s
    )


    target_link_libraries(iperf3
        iperf3_api
        lwIP_api
        os_api
        wifi_api
    )

    add_custom_command(TARGET iperf3 POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:iperf3> ${PROJECT_SOURCE_DIR}/NSPE/WIFI_IOT/lib/GCC/libiperf3.a
    )
endif()