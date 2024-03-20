cmake_minimum_required(VERSION 3.5)

project(PathPlanning VERSION 1.0 LANGUAGES CXX)

# Set the C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
# set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")


# Find Python 3 and optionally NumPy
find_package(Python3 COMPONENTS Interpreter Development NumPy QUIET)
if(NOT Python3_FOUND)
    # Fallback to older PythonLibs find module if Python3 components are not found
    find_package(PythonLibs REQUIRED)
    set(Python3_INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS})
    set(Python3_LIBRARIES ${PYTHON_LIBRARIES})
    # Note: This does not handle NumPy includes for the fallback case
endif()

# Include directories for header files
include_directories(
    ${Python3_INCLUDE_DIRS}
    ${Python3_NumPy_INCLUDE_DIRS}  # This may not be set if PythonLibs was used
    ${CMAKE_SOURCE_DIR}
    planner
    utils
    visual
    controller
)

# Add executable with all relevant source files
add_executable(${PROJECT_NAME} 
    main.cpp
)

# Link with Python libraries
target_link_libraries(${PROJECT_NAME}
    ${Python3_LIBRARIES}
)
