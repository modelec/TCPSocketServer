cmake_minimum_required(VERSION 3.25)
project(socketServer)

set(CMAKE_CXX_STANDARD 17)

find_package(Modelec COMPONENTS
        Utils
        CLParser
        REQUIRED
)

add_executable(socketServer main.cpp
        TCPServer.cpp
)

target_link_libraries(socketServer
        Modelec::Utils
        Modelec::CLParser
)