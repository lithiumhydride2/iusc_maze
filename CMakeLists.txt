cmake_minimum_required(VERSION 3.26)
project(iusc_maze)

set(CMAKE_CXX_STANDARD 17)

add_executable(iusc_maze main.cpp
        maze_map.h
        maze_map.cpp
        drone.h
        drone.cpp
)
