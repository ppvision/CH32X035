
# cmake_path(GET CMAKE_CURRENT_SOURCE_DIR FILENAME curr_dir)   
#  ${curr_dir}

set(APP freertos)
add_executable(${APP})

file( GLOB SRC_LIST *.c )

# message("Dir: ${curr_dir}(${CMAKE_CURRENT_SOURCE_DIR})  Source list: ${SRC_LIST}")
#include(../../tinyusb/src/CMakeLists.txt)


add_map_output(${APP})
add_extra_outputs(${APP})

target_sources(${APP} PRIVATE
    ${SRC_LIST}
        ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/croutine.c
        ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/event_groups.c
        ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/list.c
        ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/queue.c
        ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/stream_buffer.c
        ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/tasks.c
        ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/timers.c
        ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/MemMang/heap_4.c
        ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/GCC/RISC-V/port.c
        ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/GCC/RISC-V/portASM.S
)

include_directories(${APP} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/include
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/Common
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/MemMang
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/GCC/RISC-V
    ${CMAKE_CURRENT_LIST_DIR}/FreeRTOS/portable/GCC/RISC-V/chip_specific_extensions/RV32I_PFIC_no_extensions

)

target_link_libraries(${APP} "TOOLCHAIN_gcc_${TARGET_PROCESSOR}" WCH_Lib)
target_linker_file(${APP} ${CMAKE_SOURCE_DIR}/SDK/Ld/Link.ld)


# target_wrap_functions(${app_name} printf)
# target_wrap_functions(${app_name} vsprintf)
# target_link_options(${app_name} PRIVATE "-Wl,--wrap=vsprintf2")
# add_c_compile_options()



