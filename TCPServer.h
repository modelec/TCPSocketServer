#pragma once

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <vector>
#include <algorithm>
#include <map>

#include "utils.h"

struct ClientTCP
{
    std::string name;
    int socket = -1;
    bool isReady = false;
};

enum Team {
    YELLOW,
    BLUE
};

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
    bool _shouldStop = false; // Flag to indicate if the server should stop
    std::vector<ClientTCP> clients; // Store connected clients

    PinceState pinceState[3] = {NONE, NONE, NONE};
    int isRobotIdle = 0;

    struct Position {
        struct {
            float x;
            float y;
        } pos;
        float theta;
    } robotPose{};

    Position initRobotPose{};
    Position endRobotPose{};

    std::vector<ArucoTag> arucoTags;

    Team team;

public:
    explicit TCPServer(int port);

    void start();

    void acceptConnections();

    // Broadcast message to all connected clients
    void broadcastMessage(const char* message, int senderSocket = -1); // Modified method signature
    void broadcastMessage(std::string &message, int senderSocket = -1); // Modified method signature

    void sendToClient(const char* message, int clientSocket); // New method to send message to a specific client
    void sendToClient(std::string &message, int clientSocket); // New method to send message to a specific client

    void sendToClient(const char* message, const std::string& clientName); // New method to send message to a specific client
    void sendToClient(std::string &message, const std::string& clientName); // New method to send message to a specific client

    void handleMessage(const std::string& message, int clientSocket = -1);

    void clientDisconnected(int clientSocket); // New method to handle client disconnection

    void stop();

    [[nodiscard]] size_t nbClients() const;

    void checkIfAllClientsReady();

    void startGame();

    void goToAruco(const ArucoTag &arucoTag, int pince);

    void turnSolarPanel(const ArucoTag &arucoTag);

    void askArduinoPos();

    [[nodiscard]] bool shouldStop() const;

    ~TCPServer();
};
