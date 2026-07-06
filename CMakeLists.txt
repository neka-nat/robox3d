cmake_minimum_required(VERSION 3.22)
project(robox3d LANGUAGES C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

# box3d itself (as a subproject, its samples/tests are disabled automatically).
# EXCLUDE_FROM_ALL suppresses box3d's own install rules (headers/cmake config
# would otherwise pollute the wheel root); we install the library ourselves below.
set(BUILD_SHARED_LIBS ON)
add_subdirectory(external/box3d EXCLUDE_FROM_ALL)

# Batch shim (part of layer 1). Dynamically links to box3d and is placed in the same directory
add_library(robox3d_shim SHARED csrc/shim.c)
target_link_libraries(robox3d_shim PRIVATE box3d)
target_include_directories(robox3d_shim PRIVATE external/box3d/include)
if(APPLE)
	set_target_properties(robox3d_shim PROPERTIES INSTALL_RPATH "@loader_path")
else()
	set_target_properties(robox3d_shim PROPERTIES INSTALL_RPATH "$ORIGIN")
endif()

# Place the shared libraries in robox3d/_lib/ inside the wheel.
# ARCHIVE covers MSVC import libraries (.lib), which would otherwise land in
# the wheel root's default lib/ directory.
install(TARGETS box3d robox3d_shim
	LIBRARY DESTINATION robox3d/_lib
	RUNTIME DESTINATION robox3d/_lib
	ARCHIVE DESTINATION robox3d/_lib)
