#pragma once

#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>
#include <thread>
#include <utility>
#include <vector>
#include <algorithm>
#include <atomic>
#include <fstream>
#include <optional>
#include <array>

#include <Modelec/Utils.h>

enum PinceState {
    WHITE_FLOWER,
    PURPLE_FLOWER,
    FLOWER,
    NONE
};

struct ClientTCP
{
    std::string name;
    int socket = -1;
    bool isReady = false;

    ClientTCP() = default;

    explicit ClientTCP(std::string name, int socket = -1) : name(std::move(name)), socket(socket) {}
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
    std::atomic<bool> _shouldStop = false; // Flag to indicate if the server should stop
    std::vector<ClientTCP> clients; // Store connected clients

    std::array<PinceState, 3> pinceState = {NONE, NONE, NONE};

    bool brasBaisser = true;

    std::array<bool, 2> panneauCheck{false, false};

    struct Position {
        struct {
            float x;
            float y;
        } pos;
        float theta;
    };

    Position robotPose{};

    bool stopEmergency = false;

    bool handleEmergencyFlag = false;

    int arduinoSocket = -1;
    int lidarSocket = -1;

    struct Axis {
        int x, y;
    };

    Axis axisLeft{0, 0};

    double lidarDectectionAngle = 0;

    int lidarDecetionDistance = 0;

    bool alertLidar = true;

public:
    explicit TCPServer(int port);

    void start();

    void acceptConnections();

    // Broadcast message to all connected clients
    void broadcastMessage(const char* message, int senderSocket = -1); // Modified method signature
    void broadcastMessage(const std::string &message, int senderSocket = -1); // Modified method signature

    void sendToClient(const char* message, int clientSocket); // New method to send message to a specific client
    void sendToClient(const std::string &message, int clientSocket); // New method to send message to a specific client

    void sendToClient(const char* message, const std::string& clientName); // New method to send message to a specific client
    void sendToClient(const std::string &message, const std::string& clientName); // New method to send message to a specific client

    void handleMessage(const std::string& message, int clientSocket = -1);

    void clientDisconnected(int clientSocket); // New method to handle client disconnection

    void stop();

    [[nodiscard]] size_t nbClients() const;

    [[nodiscard]] bool shouldStop() const;

    void handleEmergency();

    void checkIfAllClientsReady();

    void toggleBras();

    void togglePince(int pince);

    void togglePanel(int servo_moteur);

    void percentagePanel(int servo_moteur, int percentage);

    ~TCPServer();
};
