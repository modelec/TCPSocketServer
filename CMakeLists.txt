cmake_minimum_required(VERSION 3.25)
project(socketServer)

find_package(Modelec COMPONENTS
        Utils
        CLParser
        REQUIRED
)

set(CMAKE_CXX_STANDARD 17)

add_executable(socketServer main.cpp
        TCPServer.cpp
        utils.cpp
)

target_link_libraries(socketServer
        Modelec::Utils
        Modelec::CLParser
)