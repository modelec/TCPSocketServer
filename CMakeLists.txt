cmake_minimum_required(VERSION 3.25)
project(socketServer)

set(CMAKE_CXX_STANDARD 17)

find_package(PkgConfig REQUIRED)
pkg_check_modules(TCPSocket REQUIRED TCPSocket)

add_executable(socketServer main.cpp
        ModelecServer.cpp
        ModelecServer.h)
target_link_libraries(socketServer TCPSocket)
