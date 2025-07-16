# Main CMakeLists.txt for LACS2 project
cmake_minimum_required(VERSION 3.16)

# Set the project name
set(PROJECT_NAME "lacs2_simple")

# Include the ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Define the project
project(${PROJECT_NAME})

# Set C standard
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Enable verbose output for debugging (optional)
# set(CMAKE_VERBOSE_MAKEFILE ON)

# Project-specific configurations can be added here
# For example:
# set(EXTRA_COMPONENT_DIRS "components")

# Add any custom compiler flags if needed
# add_compile_options(-Wall -Wextra -Werror)