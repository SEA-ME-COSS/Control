cmake_minimum_required(VERSION 3.5)

project(control VERSION 1.0 LANGUAGES CXX)

# Set the C++ standard to C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address")
# set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -fsanitize=address")


# Find Python 3 and optionally NumPy
find_package(PythonLibs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)


include_directories(
    ${PYTHON_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    lib
    lib/hybrid-a-star
)

FILE(GLOB NavObjects lib/hybrid-a-star/*.cpp)
FILE(GLOB Control src/*/*.cpp)

add_executable(${PROJECT_NAME} ${NavObjects} ${Control} "src/main.cpp")

target_link_libraries(${PROJECT_NAME}
    ${CERES_LIBRARIES}
    ompl
    Eigen3::Eigen
    ${PYTHON_LIBRARIES}
)