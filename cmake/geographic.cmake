#find_package (GeographicLib REQUIRED)

#include_directories(${GeographicLib_INCLUDE_DIRS})
add_subdirectory(${PROJECT_SOURCE_DIR}/third_party/GeographicLib)
include_directories(${PROJECT_SOURCE_DIR}/third_party/GeographicLib/include/)
#list(APPEND ALL_TARGET_LIBRARIES ${GeographicLib_LIBRARIES})
list(APPEND ALL_TARGET_LIBRARIES libGeographiccc)
