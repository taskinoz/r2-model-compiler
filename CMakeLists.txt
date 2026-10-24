cmake_minimum_required(VERSION 3.16)
project(r2studiomdl CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

file(GLOB_RECURSE R2C_SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_executable(r2studiomdl ${R2C_SOURCES})
target_include_directories(r2studiomdl PRIVATE src)

if(MSVC)
    target_compile_options(r2studiomdl PRIVATE /W4 /permissive- /utf-8 /wd4244 /wd4267 /wd4100)
    set_property(TARGET r2studiomdl PROPERTY MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
else()
    target_compile_options(r2studiomdl PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_link_options(r2studiomdl PRIVATE -static)
endif()

include(CTest)
if(BUILD_TESTING)
    add_executable(hull_tests tests/hull_tests.cpp src/build/hull.cpp)
    target_include_directories(hull_tests PRIVATE src)
    add_test(NAME collision_hulls COMMAND hull_tests "${CMAKE_CURRENT_SOURCE_DIR}/tests/hull_cylinder.xyz")
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if(Python3_Interpreter_FOUND)
        add_test(NAME compiler_regressions COMMAND ${CMAKE_COMMAND} -E env
            "R2C_COMPILER=$<TARGET_FILE:r2studiomdl>" ${Python3_EXECUTABLE}
            -m unittest discover -s "${CMAKE_CURRENT_SOURCE_DIR}/tests" -p "test_*.py")
        set_tests_properties(compiler_regressions PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    endif()
endif()
