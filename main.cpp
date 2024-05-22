#include "TCPServer.h"
#include <csignal>

#include <Modelec/CLParser.h>

std::atomic<bool> shouldStop = false;

void signalHandler( int signum ) {
    shouldStop = true;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    CLParser clParser(argc, argv);

    int port = clParser.getOption<int>("port", 8080);

    TCPServer server(port);

    server.start();

    while (!server.shouldStop() && !shouldStop) {
        usleep(500'000);
    }

    return 0;
}