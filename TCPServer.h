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

#include "utils.h"

#define MAX_SPEED 200
#define MIN_SPEED 150

struct ClientTCP
{
    std::string name;
    int socket = -1;
    bool isReady = false;

    ClientTCP() = default;

    explicit ClientTCP(std::string name, int socket = -1) : name(std::move(name)), socket(socket) {}
};

enum Team {
    BLUE,
    YELLOW,
    TEST,
};

enum StratPattern {
    TURN_SOLAR_PANNEL_1,
    TURN_SOLAR_PANNEL_2,
    TURN_SOLAR_PANNEL_3,
    TAKE_FLOWER_BOTTOM,
    TAKE_FLOWER_TOP,
    DROP_PURPLE_FLOWER,
    DROP_WHITE_FLOWER_J1,
    DROP_WHITE_FLOWER_J2,
    GO_END,
    GET_LIDAR_POS,
    CHECKPOINT_MIDDLE,
    CHECKPOINT_TRANSITION_SOLAR_PANEL_FLOWER,
    TAKE_3_PLANT_BOTTOM_1,
    TAKE_3_PLANT_BOTTOM_2,
    TAKE_3_PLANT_TOP_1,
    TAKE_3_PLANT_TOP_2,
    DROP_FLOWER_J1,
    DROP_FLOWER_J2,
    REMOVE_POT_J2,
    DROP_FLOWER_BASE_1,
    DROP_FLOWER_BASE_2,
    SLEEP_1S,
    SLEEP_5S,
    SLEEP_10S,
    ROTATE_0,
    ROTATE_270,
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

    std::vector<StratPattern> stratPatterns = {
        TURN_SOLAR_PANNEL_1,
        TURN_SOLAR_PANNEL_2,
        TURN_SOLAR_PANNEL_3,
        CHECKPOINT_TRANSITION_SOLAR_PANEL_FLOWER,

        /*TAKE_FLOWER_BOTTOM,
        TAKE_FLOWER_BOTTOM,
        TAKE_FLOWER_BOTTOM,*/

        /*TAKE_3_PLANT_BOTTOM_1,
        DROP_FLOWER_BASE_2,*/

        TAKE_3_PLANT_BOTTOM_1,
        GET_LIDAR_POS,
        REMOVE_POT_J2,
        DROP_FLOWER_J2,
        ROTATE_270,

        /*DROP_PURPLE_FLOWER,
        DROP_WHITE_FLOWER_J1,*/
        // GET_LIDAR_POS,
        /*TAKE_FLOWER_TOP,
        TAKE_FLOWER_TOP,
        TAKE_FLOWER_TOP,*/

        TAKE_3_PLANT_TOP_1,
        GET_LIDAR_POS,
        DROP_FLOWER_J1,

        /*DROP_PURPLE_FLOWER,
        DROP_WHITE_FLOWER_J2,*/
        // GET_LIDAR_POS,
        /*TAKE_FLOWER_TOP,
        TAKE_FLOWER_TOP,
        TAKE_FLOWER_TOP,*/
        //TAKE_3_PLANT_TOP,
        /*DROP_WHITE_FLOWER_J2,
        DROP_PURPLE_FLOWER,*/

        TAKE_3_PLANT_TOP_2,
        DROP_FLOWER_BASE_1,

        GO_END
    };

    // This is the index of the current pattern
    int whereAmI = 0;

    bool stopEmergency = false;
    bool handleEmergencyFlag = false;

    bool awaitForLidar = false;

    std::thread gameThread;

    int lidarSocket = -1;
    int arduinoSocket = -1;

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

    void handleArucoTag(const ArucoTag &tag);

    std::optional<ArucoTag> getBiggestArucoTag(float borneMinX, float borneMaxX, float borneMinY, float borneMaxY);

    std::optional<ArucoTag> getMostCenteredArucoTag(float borneMinX, float borneMaxX, float borneMinY, float borneMaxY);

    std::vector<PinceState> getNotFallenFlowers() const;

    void handleEmergency(int distance, double angle);

    /*
     * Start Strategy function
    */
    void goAndTurnSolarPanel(StratPattern sp);

    void findAndGoFlower(StratPattern sp);

    void goEnd();

    void dropPurpleFlowers();

    void dropWhiteFlowers(StratPattern sp);

    void getLidarPos();

    void checkpoint(StratPattern sp);

    void dropJardiniereFlowers(StratPattern sp);

    void dropBaseFlowers(StratPattern sp);

    void go3Plants(StratPattern sp);

    void removePot(StratPattern sp);
    /*
     *  End Strategy function
     */

    void startTestAruco(int pince);

    // Call to broadcast
    void setSpeed(int speed);

    void setMaxSpeed();

    void setMinSpeed();

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

    template<class X, class Y, class Z>
    void setPosition(X x, Y y, Z theta, const std::string &toSend);

    template<class X>
    void setPosition(std::array<X, 3> data, const std::string &toSend);

    void setPosition(Position pos, const std::string &toSend);

    void openPince(int pince);

    void fullyOpenPince(int pince);

    void middlePince(int pince);

    void closePince(int pince);

    void baisserBras();

    void leverBras();

    void transportBras();

    void checkPanneau(int servo_moteur);

    void uncheckPanneau(int servo_moteur);

    void askLidarPosition();

    void sendPoint(int point);

    void setTeam(Team team);

    ~TCPServer();
};
