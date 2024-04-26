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
#include <map>
#include <cmath>
#include <fstream>
#include <optional>

#include "utils.h"

struct ClientTCP
{
    std::string name;
    int socket = -1;
    bool isReady = false;

    ClientTCP() = default;

    explicit ClientTCP(std::string name, int socket = -1) : name(std::move(name)), socket(socket) {}
};

enum Team {
    YELLOW,
    BLUE,
    TEST
};

enum StratPattern {
    TURN_SOLAR_PANNEL_1,
    TURN_SOLAR_PANNEL_2,
    TURN_SOLAR_PANNEL_3,
    TAKE_FLOWER_BOTTOM,
    TAKE_FLOWER_TOP,
    DROP_FLOWER,
    GO_END,
    GET_LIDAR_POS,
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
    int isRobotIdle = 0;

    bool gameStarted = false;

    std::chrono::time_point<std::chrono::system_clock> gameStart;

    int speed = 0;

    struct Position {
        struct {
            float x;
            float y;
        } pos;
        float theta;
    };

    Position robotPose{};
    Position initRobotPose{};
    Position endRobotPose{};
    Position lidarCalculatePos{};

    std::vector<ArucoTag> arucoTags;

    Team team;

    // TODO drop white flower in other jardienière (if time)
    std::vector<StratPattern> stratPatterns = {
        TURN_SOLAR_PANNEL_1,
        TURN_SOLAR_PANNEL_2,
        TURN_SOLAR_PANNEL_3,
        GET_LIDAR_POS,
        TAKE_FLOWER_BOTTOM,
        TAKE_FLOWER_BOTTOM,
        TAKE_FLOWER_BOTTOM,
        GET_LIDAR_POS,
        DROP_FLOWER,
        GET_LIDAR_POS,
        TAKE_FLOWER_TOP,
        TAKE_FLOWER_TOP,
        TAKE_FLOWER_TOP,
        GET_LIDAR_POS,
        DROP_FLOWER,
        GET_LIDAR_POS,
        /* TAKE_FLOWER_TOP,
        TAKE_FLOWER_TOP,
        TAKE_FLOWER_TOP,
        DROP_FLOWER, */
        GO_END
    };

    // This is the index of the current pattern
    int whereAmI = 0;

    bool stopEmergency = false;
    bool handleEmergencyFlag = false;

    bool awaitForLidar = false;

    std::thread gameThread;

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

    void checkIfAllClientsReady();

    void startGame();

    void startGameTest();

    void goToAruco(const ArucoTag &arucoTag, int pince);

    void askArduinoPos();

    [[nodiscard]] bool shouldStop() const;

    void awaitRobotIdle();

    void handleArucoTag(ArucoTag &tag);

    std::optional<ArucoTag> getBiggestArucoTag(float borneMinX, float borneMaxX, float borneMinY, float borneMaxY);

    void handleEmergency(int distance, double angle);

    /*
     * Start Strategy function
    */
    void goAndTurnSolarPannel(StratPattern sp);

    void findAndGoFlower(StratPattern sp);

    void goEnd();

    void dropFlowers();

    void getLidarPos();
    /*
     *  End Strategy function
     */

    void startTestAruco(int pince);

    // Call to broadcast
    void setSpeed(int speed);

    template<class X, class Y>
    void go(X x, Y y);

    template<class X>
    void go(std::array<X, 2> data);

    template<class X>
    void rotate(X angle);

    template<class X, class Y>
    void transit(X x,Y y, int endSpeed);

    template<class X>
    void transit(std::array<X, 2> data, int endSpeed);

    template<class X, class Y, class Z>
    void setPosition(X x, Y y, Z theta, int clientSocket = -1);

    template<class X>
    void setPosition(std::array<X, 3> data, int clientSocket = -1);

    void setPosition(Position pos, int clientSocket = -1);

    void setPosition(Position pos, const std::string &toSend);

    void openPince(int pince);

    void middlePince(int pince);

    void closePince(int pince);

    void baisserBras();

    void leverBras();

    void transportBras();

    void checkPanneau(int servo_moteur);

    void uncheckPanneau(int servo_moteur);

    void askLidarPosition();

    ~TCPServer();
};
