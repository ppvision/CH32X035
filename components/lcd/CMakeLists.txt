if (NOT TARGET lcd)
    add_library(lcd INTERFACE)
    target_sources(lcd INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/fonts.c
            ${CMAKE_CURRENT_LIST_DIR}/st7735.c
#            ${CMAKE_CURRENT_LIST_DIR}/st7789.c
            ${CMAKE_CURRENT_LIST_DIR}/lcd_api.c
    )
    target_include_directories(lcd INTERFACE ${CMAKE_CURRENT_LIST_DIR}/)
endif()
