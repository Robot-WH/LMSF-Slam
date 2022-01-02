
add_subdirectory(${PROJECT_SOURCE_DIR}/third_part/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/third_part/GeographicLib/include/)
list(APPEND ALL_TARGET_LIBRARIES libGeographiccc)  # ALL_TARGET_LIBRARIES 为列表, 即将 libGeographiccc 添加到
                                                                                                                   # ALL_TARGET_LIBRARIES 后面 