cmake_minimum_required(VERSION 3.14)

project(livox_sdk2)

set(CMAKE_CXX_STANDARD 11)

# Keep standalone SDK builds compatible with upstream, while allowing the ROS
# driver to embed only the static SDK without adding SDK install rules.
option(LIVOX_SDK2_BUILD_SHARED "Build the shared Livox SDK library" ON)
option(LIVOX_SDK2_BUILD_SAMPLES "Build Livox SDK sample applications" ON)
option(LIVOX_SDK2_INSTALL "Install Livox SDK libraries and headers" ON)
find_package(Threads REQUIRED)

message(STATUS "main project dir: " ${PROJECT_SOURCE_DIR})

if (CMAKE_CROSSCOMPILING)
	set(THREADS_PTHREAD_ARG
		"PLEASE_FILL_OUT-FAILED_TO_RUN"
		CACHE STRING "Result from TRY_RUN" FORCE)
endif()

add_subdirectory(sdk_core)
if(LIVOX_SDK2_BUILD_SAMPLES)
  add_subdirectory(samples)
endif()
