# CMakeLists.txt
cmake_minimum_required(VERSION 3.20)
project(mrele)

set(CMAKE_CXX_STANDARD 20)

include_directories(include)

add_executable(mrele

        main.cpp
        sqlite3.c
        sqlite3.h
        include/mrele/motor.h
        include/mrele/ray_state.h
        include/mrele/optical_element.h
        include/mrele/risley_prism.h
        include/mrele/virtual_testbed.h
        include/mrele/database_logger.h
        include/mrele/quadrant_sensor.h
        src/motor.cpp
        src/risley_prism.cpp
        src/quadrant_sensor.cpp
        src/database_logger.cpp
        src/virtual_testbed.cpp
        include/mrele/test_scenario.h
)
set_source_files_properties(sqlite3.c PROPERTIES LANGUAGE C)