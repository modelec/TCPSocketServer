#include "TCPServer.h"
#include <Modelec/CLParser.h>
#include <csignal>

std::atomic<bool> shouldStop = false;

void signalHandler( int signum ) {

    shouldStop = true;

    exit(signum);
}

int main(int argc, char* argv[]) {
    signal(SIGINT, signalHandler);

    CLParser clParser(argc, argv);

    int port = clParser.getOption<int>("port", 8080);

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