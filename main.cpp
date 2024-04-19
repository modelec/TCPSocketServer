#include "TCPServer.h"
#include <csignal>

std::atomic<bool> shouldStop = false;

void signalHandler( int signum ) {

    shouldStop = true;

    exit(signum);
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    int port = 8080;
    if (argc >= 2) {
        port = std::stoi(argv[1]);
    }

    TCPServer server(port);

    try {
        server.start();

        while (!server.shouldStop() && !shouldStop) {
            usleep(500'000);
        }

        server.stop();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}