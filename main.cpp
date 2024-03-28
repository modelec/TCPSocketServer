#include "TCPServer.h"

int main(int argc, char* argv[]) {

    int port = 8080;
    if (argc >= 2) {
        port = std::stoi(argv[1]);
    }

    TCPServer server(port);

    try {
        server.start();

        while (true) {
            sleep(1);

            std::string message;
            std::cout << "Enter message ('quit' to exit): ";
            std::getline(std::cin, message);

            if (message == "quit") {
                server.stop();
                break;
            }

            server.broadcastMessage(message.c_str());

        }

        server.stop();
    } catch (const std::exception& ex) {
        std::cerr << "Error: " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}