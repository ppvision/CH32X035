


add_library(WCH_Lib INTERFACE)
target_include_directories(WCH_Lib INTERFACE 
    ${CMAKE_SOURCE_DIR}/config
    ${CMAKE_CURRENT_LIST_DIR}/Core
    ${CMAKE_CURRENT_LIST_DIR}/Debug
    ${CMAKE_CURRENT_LIST_DIR}/Peripheral/inc;

)

file( GLOB CORE_SRC  Core/*.c )
file( GLOB DEBUG_SRC  Debug/*.c )
file( GLOB DEVICE_SRC Peripheral/src/*.c )
file( GLOB STARTUP_SRC Startup/startup_ch32x035.S )
# file(GLOB_RECURSE CMSIS_SRC  RELATIVE ${CMAKE_CURRENT_LIST_DIR} "CMSIS/*.c")
# file(GLOB_RECURSE DEBUG_SRC  RELATIVE ${CMAKE_CURRENT_LIST_DIR} "Debug/*.c")
# file(GLOB_RECURSE DEVICE_SRC RELATIVE ${CMAKE_CURRENT_LIST_DIR} "StdPeriphDriver/src/*.c")

SET(SRC_LIST 
    ${CORE_SRC}
    ${DEBUG_SRC}
    ${DEVICE_SRC}
    ${STARTUP_SRC}
)

message("WCH_Lib:SRC_LIST ${SRC_LIST}")

target_sources(WCH_Lib INTERFACE
	${SRC_LIST}
)

