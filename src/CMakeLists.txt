FILE(GLOB_RECURSE app_sources 
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../resources/*.cpp"
)

idf_component_register(
    SRCS ${app_sources}
    INCLUDE_DIRS "." "../resources"
    REQUIRES vl53l0x
)
