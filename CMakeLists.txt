cmake_minimum_required(VERSION 3.25)
project(socketServer)

set(CMAKE_CXX_STANDARD 17)

add_executable(socketServer main.cpp
        TCPServer.cpp
        TCPServer.h)
