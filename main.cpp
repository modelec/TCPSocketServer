#include "TCPServer.h"

int main(int argc, char* argv[]) {

    int port = 8080;
    if (argc >= 2) {
        port = std::stoi(argv[1]);
    }

    TCPServer server(port);

    try {
        server.start();

        while (!server.shouldStop()) {
            usleep(1'000'000);
        }

        server.stop();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}