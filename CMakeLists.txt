cmake_minimum_required(VERSION 3.5)
SET(TARGET_PROCESSOR riscv32)



#SET(TOOLCHAIN_DIR /Users/bright/.toolchain/xpack-riscv-none-elf-gcc-12.2.0-3)
#SET(TARGET_TRIPLET "riscv-none-elf")

SET(TOOLCHAIN_DIR /Users/bright/.toolchain/xpack-riscv-none-embed-gcc-8.2.0)
SET(TARGET_TRIPLET "riscv-none-embed")



include(cmake/util.cmake)
include(cmake/toolchain/toolchain_riscv.cmake)


project(CH32V C ASM)


option(USB_CONSOLE "build for USB console, otherwise UART" OFF)
option(BUILD_TYPE  "build Debug version" ON)



if(BUILD_TYPE)
	SET(CMAKE_BUILD_TYPE Debug)
endif()





set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG   ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/release)


# import some lib
# include(lib/cmake_libs.cmake)
include_directories(config)

add_subdirectory(SDK)
add_subdirectory(components/lcd)
add_subdirectory(Apps/gpio)
add_subdirectory(Apps/SimulateCDC)
add_subdirectory(Apps/FreeRTOS)


# include(tinyusb.cmake)


# add_executable(${APP_NAME} ${SRC_LIST})
# add_map_output(${APP_NAME})
# add_extra_outputs(${APP_NAME})

# target_include_directories(${APP_NAME} PRIVATE 
# 	App
# )


# # target_wrap_functions(${APP_NAME} printf)
# # target_wrap_functions(${APP_NAME} vsprintf)
# # target_link_options(${APP_NAME} PRIVATE "-Wl,--wrap=vsprintf2")
# # add_c_compile_options()


# target_link_libraries(${APP_NAME} "TOOLCHAIN_gcc_${TARGET_PROCESSOR}" WCH_Lib)

# target_linker_file(${APP_NAME} ${CMAKE_CURRENT_LIST_DIR}/cmake/ld/chf2xx.ld)
