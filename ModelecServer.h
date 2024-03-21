#pragma once

#include <TCPSocket/TCPServer.hpp>

class ModelecServer : public TCPServer {

public:
    void acceptConnections() override;

    ~ModelecServer() override;

    explicit ModelecServer(int port);

    void handleMessage(const std::string& message, int clientSocket) override;
};
