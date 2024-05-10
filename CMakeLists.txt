cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(helloworld)

set(OpenCASCADE_DIR "C:/Program Files/OCCT/cmake")
find_package(OpenCASCADE)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)

set(CMAKE_TOOLCHAIN_FILE "C:/vcpkg/scripts/buildsystems/vcpkg.cmake")
find_package(CGAL CONFIG REQUIRED)

include_directories(${OpenCASCADE_INCLUDE_DIR})
link_directories("${OpenCASCADE_LIBRARY_DIR}i")

file(GLOB SRC_CPP ${CMAKE_SOURCE_DIR}/*.cpp)
file(GLOB SRC_H ${CMAKE_SOURCE_DIR}/*.h)

add_executable(${PROJECT_NAME} ${SRC_CPP} ${SRC_H})

target_link_libraries(helloworld PRIVATE CGAL::CGAL)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCASCADE_LIBRARIES})

file(GLOB DLL_FILES "${OpenCASCADE_BINARY_DIR}i/*.dll")
foreach(DLL_FILE ${DLL_FILES})
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
                       COMMAND ${CMAKE_COMMAND} -E copy_if_different
                               ${DLL_FILE} $<TARGET_FILE_DIR:${PROJECT_NAME}>)
endforeach()