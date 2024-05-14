#include "TCPServer.h"
#include <csignal>

std::atomic<bool> shouldStop = false;

void signalHandler( int signum ) {
    shouldStop = true;
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    int port = 8080;
    if (argc >= 2) {
        port = std::stoi(argv[1]);
    }

    TCPServer server(port);

    server.start();

    while (!server.shouldStop() && !shouldStop) {
        usleep(500'000);
    }

    server.stop();

    return 0;
}