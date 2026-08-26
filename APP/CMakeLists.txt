target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/arm/Src/gimbal_task.c
    ${CMAKE_CURRENT_LIST_DIR}/comm2motor/Src/bldc_motor.c
    ${CMAKE_CURRENT_LIST_DIR}/debug/Src/debug_task.c
)

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/arm/Inc
    ${CMAKE_CURRENT_LIST_DIR}/comm2motor/Inc
    ${CMAKE_CURRENT_LIST_DIR}/debug/Inc
)
