
# cmake_path(GET CMAKE_CURRENT_SOURCE_DIR FILENAME curr_dir)   
#  ${curr_dir}

set(APP gpio)
add_executable(${APP})

set_target_properties(${APP} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})



file( GLOB SRC_LIST *.c )
# message("Dir: ${curr_dir}(${CMAKE_CURRENT_SOURCE_DIR})  Source list: ${SRC_LIST}")
#include(../../tinyusb/src/CMakeLists.txt)




add_map_output(${APP})
add_extra_outputs(${APP})


add_compile_definitions(-DLV_USE_DEMO_BENCHMARK=1)


include_directories(${APP} PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_SOURCE_DIR}/config
)

target_sources(${APP} PRIVATE
    ${SRC_LIST}
)



target_link_libraries(${APP} "TOOLCHAIN_gcc_${TARGET_PROCESSOR}" WCH_Lib lcd )
target_linker_file(${APP} ${CMAKE_SOURCE_DIR}/SDK/Ld/Link.ld)


# target_wrap_functions(${app_name} printf)
# target_wrap_functions(${app_name} vsprintf)
# target_link_options(${app_name} PRIVATE "-Wl,--wrap=vsprintf2")
# add_c_compile_options()



