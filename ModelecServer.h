#pragma once

#include <TCPSocket/TCPServer.hpp>

class ModelecServer : public TCPServer {

public:
    ModelecServer(int port);

    void handleMessage(const std::string& message, int clientSocket) override;
};
