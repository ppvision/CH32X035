
# cmake_path(GET CMAKE_CURRENT_SOURCE_DIR FILENAME curr_dir)   
#  ${curr_dir}

set(APP cdc)
add_executable(${APP})

file( GLOB SRC_LIST *.c )

# message("Dir: ${curr_dir}(${CMAKE_CURRENT_SOURCE_DIR})  Source list: ${SRC_LIST}")
#include(../../tinyusb/src/CMakeLists.txt)


add_map_output(${APP})
add_extra_outputs(${APP})


add_tinyusb(${APP})

target_sources(${APP} PRIVATE
    ${SRC_LIST}
)


include_directories(${APP} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(${APP} "TOOLCHAIN_gcc_${TARGET_PROCESSOR}" WCH_Lib)
target_linker_file(${APP} ${CMAKE_SOURCE_DIR}/cmake/ld/riscv/ch32x05.ld)


# target_wrap_functions(${app_name} printf)
# target_wrap_functions(${app_name} vsprintf)
# target_link_options(${app_name} PRIVATE "-Wl,--wrap=vsprintf2")
#



