#pragma once

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <vector>
#include <algorithm>

class TCPServer; // Forward declaration

class ClientHandler {
private:
    int clientSocket;
    TCPServer* server; // Reference to the TCPServer instance

public:
    explicit ClientHandler(int clientSocket, TCPServer* server);

    void handle();

    void processMessage(const std::string& message);

    void closeConnection();
};

class TCPServer {
private:
    int serverSocket;
    std::vector<std::thread> clientThreads;
    std::vector<int> clientSockets; // Store connected client sockets
    int connectedClients = 0; // Track the number of connected clients
    bool shouldStop = false; // Flag to indicate if the server should stop

public:
    explicit TCPServer(int port);

    void start();

    void acceptConnections();

    // Broadcast message to all connected clients
    void broadcastMessage(const char* message, int senderSocket = -1); // Modified method signature

    void handleMessage(const std::string& message, int clientSocket = -1);

    void clientDisconnected(int clientSocket); // New method to handle client disconnection

    void stop();

    int nbClients();

    ~TCPServer();
};
