#include "TCPServer.h"

ClientHandler::ClientHandler(int clientSocket, TCPServer* server) : clientSocket(clientSocket), server(server) {};

void ClientHandler::handle() {
    std::string buffer;
    buffer.reserve(8192); // Pre-allocate memory to avoid frequent allocations

    while (true) {
        char tempBuffer[8192] = {0};
        ssize_t valread = recv(clientSocket, tempBuffer, sizeof(tempBuffer), 0);

        if (valread > 0) {
            buffer.append(tempBuffer, valread);

            if (buffer == "quit") {
                std::cerr << "Client requested to quit. Closing connection." << std::endl;
                break;
            }

            std::vector<std::string> messages = TCPUtils::split(buffer, "\n");
            for (const std::string& message : messages) {
                processMessage(message);
            }

            buffer.clear();
        } else if (valread == 0) {
            std::cout << "Client disconnected. " << clientSocket << std::endl;
            break; // Client disconnected
        } else {
            std::cerr << "Failed to receive data." << this->clientSocket << std::endl;
            break; // Error in receiving data
        }
    }

    closeConnection();
}

void ClientHandler::processMessage(const std::string& message) {
    server->handleMessage(message, clientSocket);
}

void ClientHandler::closeConnection() {
    close(clientSocket);
    server->clientDisconnected(clientSocket); // Inform the server that the client has disconnected
}

TCPServer::TCPServer(int port) : team(TEST)
{
    this->robotPose = {500, 500, -3.1415/2};

    serverSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (serverSocket == -1) {
        std::cerr << "Socket creation failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(serverSocket, reinterpret_cast<struct sockaddr*>(&address), sizeof(address)) == -1) {
        std::cerr << "Binding failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    if (listen(serverSocket, 5) == -1) {
        std::cerr << "Listening failed" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "Server started on port " << port << std::endl;

    clients.reserve(6);

    clients.emplace_back("tirette");
    clients.emplace_back("aruco");
    clients.emplace_back("ihm");
    clients.emplace_back("lidar");
    clients.emplace_back("arduino");
    clients.emplace_back("servo_moteur");
    // clients.emplace_back("point");

}

void TCPServer::acceptConnections()
{
    while (!_shouldStop) {
        sockaddr_in clientAddress{};
        int addrlen = sizeof(clientAddress);
        int clientSocket =
            accept(serverSocket, reinterpret_cast<struct sockaddr*>(&clientAddress), reinterpret_cast<socklen_t*>(&addrlen));
        if (clientSocket == -1) {
            std::cerr << "Accepting connection failed" << std::endl;
            continue;
        }
        std::cout << "Connection accepted" << std::endl;


        // Add the client socket to the list
        clientSockets.push_back(clientSocket);
        connectedClients++;

        // Handle client connection in a separate thread
        clientThreads.emplace_back(&ClientHandler::handle, ClientHandler(clientSocket, this));
        // Clean up finished threads
        clientThreads.erase(std::remove_if(clientThreads.begin(), clientThreads.end(), [](std::thread &t) {
            return !t.joinable();
        }), clientThreads.end());
    }
}

void TCPServer::handleMessage(const std::string& message, int clientSocket)
{
    std::cout << message << std::endl;

    std::vector<std::string> tokens = TCPUtils::split(message, ";");

    if (tokens.size() != 4)
    {
        std::cerr << "Invalid message format, token size : " << std::to_string(tokens.size()) << " from message : " << message << std::endl;
        return;
    }
    if (TCPUtils::contains(tokens[2], "stop proximity")) {
        if (!gameStarted) return;

        std::vector<std::string> args = TCPUtils::split(tokens[3], ",");

        if (stoi(args[0]) == -1) return;

        this->broadcastMessage("strat;arduino;clear;1\n");

        this->stopEmergency = true;

        // if (!handleEmergencyFlag) {
            // std::thread([this, args]() { this->handleEmergency(std::stoi(args[0]), std::stod(args[1]) / 100); }).detach();
        // }
    }
    else if (tokens[1] != "strat") {
        this->broadcastMessage(message, clientSocket);
    }
    // EMERGENCY
    else if (tokens[0] == "tirette" && tokens[2] == "set state") {
        this->broadcastMessage(message, clientSocket);
    }
    else if (tokens[2] == "ready") {
        for (ClientTCP& client : clients)
        {
            if (client.name == tokens[0])
            {
                client.isReady = true;
                client.socket = clientSocket;
                if (TCPUtils::contains(client.name, "lidar")) {
                    this->lidarSocket = clientSocket;
                }
                std::cout << client.socket << " | " << client.name << " is ready" << std::endl;
                break;
            }
        }
        checkIfAllClientsReady();
    }
    else if (tokens[2] == "get pos") {
        this->setPosition(this->robotPose, clientSocket);
    }
    else if (tokens[2] == "get speed") {
        this->sendToClient("strat;" + tokens[0] + ";set speed;" + std::to_string(this->speed) + "\n", clientSocket);
    }
    else if (tokens[0] == "lidar" && tokens[2] == "set pos") {
        std::vector<std::string> args = TCPUtils::split(tokens[3], ",");
        // TODO replace angle with the real angle calculated by the lidar when working
        this->lidarCalculatePos = {std::stof(args[0]), std::stof(args[1]), /*std::stof(args[2]) / 100*/ this->robotPose.theta};
        if (lidarCalculatePos.pos.x == -1 || lidarCalculatePos.pos.y == -1) {
            if (lidarGetPosTimeout > 10) {
                awaitForLidar = false;
            }
            lidarGetPosTimeout++;
            this->askLidarPosition();
        }
        else {
            this->setPosition(this->lidarCalculatePos);
            usleep(100'000);
            this->setPosition(this->lidarCalculatePos);
            awaitForLidar = false;
        }
    }
    else if (tokens[0] == "ihm") {
        if (tokens[2] == "spawn") {
            int spawnPointNb = std::stoi(tokens[3]);
            std::array<float, 3> spawnPoint{};
            std::array<float, 3> finishPoint{};

            switch (spawnPointNb) {
                case 3:
                    this->setTeam(BLUE);
                    spawnPoint[0] = 250;
                    spawnPoint[1] = 1800;
                    spawnPoint[2] = 0;

                    finishPoint[0] = 400;
                    finishPoint[1] = 500;
                    finishPoint[2] = PI / 2;

                    // For test

                    /*spawnPoint[0] = 500;
                    spawnPoint[1] = 1000;
                    spawnPoint[2] = 0;*/

                    /*finishPoint[0] = 400;
                    finishPoint[1] = 1790;
                    finishPoint[2] = 0;*/
                    break;
                case 6:
                    this->setTeam(YELLOW);
                    spawnPoint[0] = 2750;
                    spawnPoint[1] = 1800;
                    spawnPoint[2] = PI;

                    finishPoint[0] = 2600;
                    finishPoint[1] = 500;
                    finishPoint[2] = PI / 2;
                    break;

                default:
                    this->team = TEST;
                    spawnPoint[0] = 1200;
                    spawnPoint[1] = 1800;
                    spawnPoint[2] = PI / 2;

                    finishPoint[0] = 1200;
                    finishPoint[1] = 1800;
                    finishPoint[2] = PI / 2;
                    break;
            }

            std::ofstream file("end_point.txt");
            file << finishPoint[0] << " " << finishPoint[1];
            file.close();

            this->robotPose = {spawnPoint[0], spawnPoint[1], spawnPoint[2]};
            this->initRobotPose = {spawnPoint[0], spawnPoint[1], spawnPoint[2]};
            this->endRobotPose = {finishPoint[0], finishPoint[1], finishPoint[2]};

            for (int j = 0; j < 3; j++) {
                this->setPosition(this->initRobotPose);
                usleep(100'000);
            }
        }
        else if (tokens[1] == "strat" && tokens[2] == "start")
        {
            if (this->gameStarted) {
                return;
            }

            this->broadcastMessage(message, clientSocket);

            this->gameStarted = true;

            this->gameStart = std::chrono::system_clock::now();

            this->setSpeed(200);

            switch (this->team) {
                case BLUE:
                case YELLOW:
                    this->gameThread = std::thread([this]() { this->startGame(); });
                    break;
                case TEST:
                    this->gameThread = std::thread([this]() { this->startGameTest(); });
                    break;
            }

            this->gameThread.detach();
        }
    }
    else if (tokens[0] == "aruco" && tokens[2] == "get aruco") {
        std::string arucoResponse = tokens[3];
        if (arucoResponse != "404") {
            std::vector<std::string> aruco = TCPUtils::split(arucoResponse, ",");
            for (int i = 0; i < aruco.size() - 1; i += 7) {
                ArucoTag tag;
                tag.setId(std::stoi(aruco[i]));
                tag.setName(aruco[i + 1]);

                tag.setPos(std::stof(aruco[i + 2]), std::stof(aruco[i + 3]));
                tag.setRot(std::stof(aruco[i + 4]), std::stof(aruco[i + 5]), std::stof(aruco[i + 6]));

                // std::cout << tag << std::endl;

                handleArucoTag(tag);
            }
            // Broadcast the aruco tag to all clients
            this->broadcastMessage(message, clientSocket);
        }
    }
    else if (tokens[0] == "arduino") {
        if (tokens[2] == "set state") {
            if (TCPUtils::startWith(tokens[3], "0")) {
                this->isRobotIdle++;
            }
        } else if (tokens[2] == "set speed") {
            this->speed = std::stoi(tokens[3]);
        } else if (tokens[2] == "set pos") {
            std::vector<std::string> pos = TCPUtils::split(tokens[3], ",");
            this->robotPose = {std::stof(pos[0]), std::stof(pos[1]), std::stof(pos[2]) / 100};
            if (!awaitForLidar) {
                this->setPosition(this->robotPose, lidarSocket);
            }
        }
    } else if (tokens[2] == "test aruco") {
        int pince = std::stoi(tokens[3]);

        std::thread([this, pince]() { this->startTestAruco(pince); }).detach();
    }
    // std::cout << "Received: " << message << std::endl;
}

void TCPServer::broadcastMessage(const char* message, int senderSocket)
{
    std::string temp = std::string(message);
    if (temp[temp.size() - 1] != '\n') {
        temp += '\n';
    }

    for (int clientSocket : clientSockets) {
        if (clientSocket != senderSocket) { // Exclude the sender's socket
            send(clientSocket, temp.c_str(), temp.length(), 0);
        }
    }
}

void TCPServer::broadcastMessage(const std::string &message, int senderSocket) {
    std::string temp = const_cast<std::string&>(message);
    if (temp[temp.size() - 1] != '\n') {
        temp += '\n';
    }

    for (int clientSocket : clientSockets) {
        if (clientSocket != senderSocket) { // Exclude the sender's socket
            send(clientSocket, temp.c_str(), temp.length(), 0);
        }
    }
}

void TCPServer::sendToClient(const std::string &message, int clientSocket) {
    std::string temp = const_cast<std::string&>(message);
    if (temp[temp.size() - 1] != '\n') {
        temp += '\n';
    }

    for (int socket : clientSockets) {
        if (socket == clientSocket) {
            send(socket, temp.c_str(), temp.size(), 0);
            break;
        }
    }
}

void TCPServer::sendToClient(const char *message, int clientSocket) {
    std::string temp = std::string(message);
    if (temp[temp.size() - 1] != '\n') {
        temp += '\n';
    }

    for (int socket : clientSockets) {
        if (socket == clientSocket) {
            send(socket, temp.c_str(), temp.size(), 0);
            break;
        }
    }
}

void TCPServer::sendToClient(const std::string &message, const std::string &clientName) {
    this->sendToClient(message.c_str(), clientName);
}

void TCPServer::sendToClient(const char *message, const std::string &clientName) {
    for (auto & [name, socket, ready] : clients) {
        if (name == clientName) {
            send(socket, message, strlen(message), 0);
            break;
        }
    }
}

bool TCPServer::shouldStop() const {
    return _shouldStop;
}

void TCPServer::clientDisconnected(const int clientSocket) {
    // Remove the disconnected client's socket
    clientSockets.erase(std::remove(clientSockets.begin(), clientSockets.end(), clientSocket), clientSockets.end());
    // Decrement the count of connected clients
    connectedClients--;
}

void TCPServer::stop() {
    _shouldStop = true;
    // Close all client sockets
    for (int clientSocket : clientSockets) {
        close(clientSocket);
    }
    // Join all threads before exiting
    for (auto& thread : clientThreads) {
        thread.join();
    }

    if (gameStarted) {
        this->gameThread.~thread();
    }

    // Close the server socket
    close(serverSocket);
}

TCPServer::~TCPServer() {
    this->stop();
}

size_t TCPServer::nbClients() const {
    return connectedClients;
}

void TCPServer::start()
{
    std::thread([this]() { acceptConnections(); }).detach();
}

void TCPServer::checkIfAllClientsReady()
{
    bool allReady = true;
    for (auto&[name, socket, isReady] : clients)
    {
        if (!isReady)
        {
            // std::cout << name << " is not ready" << std::endl;
            allReady = false;
        }
    }

    if (allReady)
    {
        this->broadcastMessage("strat;all;ready;1\n");
        std::thread([this]() { askArduinoPos(); }).detach();
    }
}

void TCPServer::startGame() {
    gameStarted = true;
    for (int i = whereAmI; i < stratPatterns.size(); i++) {

        auto time = std::chrono::system_clock::now();
        if (time - gameStart > std::chrono::seconds(87)) {
            this->goEnd();
            return;
        }

        switch (stratPatterns[i]) {
            case TURN_SOLAR_PANNEL_1:
                goAndTurnSolarPanel(TURN_SOLAR_PANNEL_1);
                break;
            case TURN_SOLAR_PANNEL_2:
                goAndTurnSolarPanel(TURN_SOLAR_PANNEL_2);
                break;
            case TURN_SOLAR_PANNEL_3:
                goAndTurnSolarPanel(TURN_SOLAR_PANNEL_3);
                break;
            case TAKE_FLOWER_TOP:
                findAndGoFlower(TAKE_FLOWER_TOP);
               break;
            case TAKE_FLOWER_BOTTOM:
                findAndGoFlower(TAKE_FLOWER_BOTTOM);
                break;
            case GO_END:
                goEnd();
                break;
            case DROP_PURPLE_FLOWER:
                dropPurpleFlowers();
                break;
            case DROP_WHITE_FLOWER_J1:
                dropWhiteFlowers(DROP_WHITE_FLOWER_J1);
                break;
            case DROP_WHITE_FLOWER_J2:
                dropWhiteFlowers(DROP_WHITE_FLOWER_J2);
                break;
            case GET_LIDAR_POS:
                getLidarPos();
                break;
            case CHECKPOINT_MIDDLE:
                checkpoint(CHECKPOINT_MIDDLE);
                break;
            case CHECKPOINT_TRANSITION_SOLAR_PANEL_FLOWER:
                checkpoint(CHECKPOINT_TRANSITION_SOLAR_PANEL_FLOWER);
                break;
            case DROP_FLOWER_J1:
                dropJardiniereFlowers(DROP_FLOWER_J1);
                break;
            case DROP_FLOWER_J2:
                dropJardiniereFlowers(DROP_FLOWER_J2);
                break;
            case TAKE_3_PLANT_TOP_1:
                go3Plants(TAKE_3_PLANT_TOP_1);
                break;
            case TAKE_3_PLANT_BOTTOM_1:
                go3Plants(TAKE_3_PLANT_BOTTOM_1);
                break;
            case TAKE_3_PLANT_TOP_2:
                go3Plants(TAKE_3_PLANT_TOP_2);
                break;
            case TAKE_3_PLANT_BOTTOM_2:
                go3Plants(TAKE_3_PLANT_BOTTOM_2);
                break;
            case REMOVE_POT_J2:
                removePot(REMOVE_POT_J2);
                break;
            case DROP_FLOWER_BASE_1:
                dropBaseFlowers(DROP_FLOWER_BASE_1);
                break;
            case DROP_FLOWER_BASE_2:
                dropBaseFlowers(DROP_FLOWER_BASE_2);
                break;
            case SLEEP_1S:
                usleep(1'000'000);
                break;
            case SLEEP_5S:
                usleep(5'000'000);
                break;
            case SLEEP_10S:
                usleep(10'000'000);
                break;
            case ROTATE_0:
                this->rotate(0);
                if (awaitRobotIdle() < 0) return;
                break;
            case ROTATE_270:
                this->rotate(-PI/2);
                if (awaitRobotIdle() < 0) return;
                break;
        }

        whereAmI++;
    }
}

void TCPServer::startGameTest() {
    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
    this->broadcastMessage("strat;servo_moteur;fermer pince;1\n");
    this->broadcastMessage("strat;servo_moteur;fermer pince;2\n");
    this->broadcastMessage("strat;servo_moteur;ouvrir pince;0\n");
    this->broadcastMessage("strat;arduino;speed;200\n");

    arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");

    int timeout = 0;
    ArucoTag tag;
    bool found = false;
    while (!found) {
        for (const auto & arucoTag : this->arucoTags) {
            if (TCPUtils::endWith(arucoTag.name(), "flower")) {
                tag = arucoTag;
                found = true;
                break;
            }
        }

        if (!found) {
            this->broadcastMessage("start;aruco;get aruco;1");
            usleep(500'000);
            timeout++;
            if (timeout > 10) {
                return;
            }
        }
    }

    // goToAruco(tag, 1);

    if (pinceState[1] == NONE) {
        goToAruco(tag, 1);
    } else if (pinceState[2] == NONE) {
        goToAruco(tag, 2);
    } else if (pinceState[0] == NONE) {
        goToAruco(tag, 0);
    } else {
        return;
    }

    // pi/4
    this->broadcastMessage("strat;arduino;angle;314\n");
    if (awaitRobotIdle() < 0) return;

    // ReSharper disable once CppDFAUnreachableCode

    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");

    usleep(2'000'000);
    arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");

    found = false;
    timeout = 0;
    while (!found) {
        for (const auto & arucoTag : this->arucoTags) {
            if (TCPUtils::endWith(arucoTag.name(), "flower")) {
                tag = arucoTag;
                found = true;
                break;
            }
        }

        if (!found) {
            this->broadcastMessage("start;aruco;get aruco;1");
            usleep(500'000);
            timeout++;
            if (timeout > 10) {
                return;
            }
        }
    }

    if (pinceState[1] == NONE) {
        goToAruco(tag, 1);
    } else if (pinceState[2] == NONE) {
        goToAruco(tag, 2);
    } else if (pinceState[0] == NONE) {
        goToAruco(tag, 0);
    } else {
        return;
    }

    this->broadcastMessage("strat;arduino;angle;157\n");
    if (awaitRobotIdle() < 0) return;

    // this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");

    usleep(2'000'000);
    arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");

    found = false;
    timeout = 0;
    while (!found) {
        for (const auto & arucoTag : this->arucoTags) {
            if (TCPUtils::endWith(arucoTag.name(), "flower")) {
                tag = arucoTag;
                found = true;
                break;
            }
        }

        if (!found) {
            this->broadcastMessage("start;aruco;get aruco;1");
            usleep(500'000);
            timeout++;
            if (timeout > 10) {
                return;
            }
        }
    }

    if (pinceState[1] == NONE) {
        goToAruco(tag, 1);
    } else if (pinceState[2] == NONE) {
        goToAruco(tag, 2);
    } else if (pinceState[0] == NONE) {
        goToAruco(tag, 0);
    } else {
        return;
    }

    // go to jardinière

    this->broadcastMessage("strat;servo_moteur;lever bras;1\n");

    std::string toSend = "strat;arduino;go;762,300\n";
    this->broadcastMessage(toSend);
    usleep(200'000);
    if (awaitRobotIdle() < 0) return;

    this->broadcastMessage("strat;arduino;angle;157\n");
    if (awaitRobotIdle() < 0) return;

    this->broadcastMessage("strat;arduino;speed;150\n");
    this->broadcastMessage("strat;arduino;go;762,0\n");
    usleep(4'000'000);

    this->broadcastMessage("strat;servo_moteur;ouvrir pince;0\n");
    pinceState[0] = NONE;
    this->broadcastMessage("strat;servo_moteur;ouvrir pince;2\n");
    pinceState[2] = NONE;
    usleep(200'000);

    this->broadcastMessage("strat;servo_moteur;fermer pince;0\n");
    this->broadcastMessage("strat;servo_moteur;fermer pince;2\n");
    this->broadcastMessage("strat;servo_moteur;ouvrir pince;1\n");
    pinceState[1] = NONE;
    usleep(200'000);

    this->broadcastMessage("strat;arduino;speed;200\n");

    toSend = "strat;arduino;go;" + std::to_string(static_cast<int>(this->endRobotPose.pos.x)) + "," + std::to_string(static_cast<int>(this->endRobotPose.pos.y)) + "\n";
    this->broadcastMessage(toSend);
    if (awaitRobotIdle() < 0) return;

    toSend = "strat;arduino;angle;" + std::to_string(static_cast<int>(this->endRobotPose.theta * 100)) + "\n";
    this->broadcastMessage(toSend);
    if (awaitRobotIdle() < 0) return;

    // toSend = "start;arduino;angle;" + std::to_string(this->endRobotPose.theta * 100) + "\n";
    // this->broadcastMessage(toSend);

    this->broadcastMessage("strat;servo_moteur;baisser bras;1");

    this->broadcastMessage("strat;servo_moteur;clear;1");
}


void TCPServer::goToAruco(const ArucoTag &arucoTag, const int pince) {

    auto [robotPosX, robotPosY] = this->robotPose.pos;

    double theta = this->robotPose.theta;
    double decalage;
    if (pince < 0 || pince > 2) {
        return;
    }

    switch (pince) {
        case 0:
            decalage = 75;
            break;
        case 1:
            decalage = 0;
            break;
        case 2:
            decalage = -75;
            break;
        default:
            decalage = 0;
            break;
    }

    this->baisserBras();
    this->openPince(pince);

    auto [xPrime, yPrime] = arucoTag.pos();
    double roll = arucoTag.rot()[1];

    auto centerPlantX = (20 * std::cos(roll)) + xPrime - 20 + (decalage / 10);
    auto centerPlantY = (-20 * std::sin(roll)) + yPrime + decalage;

    double thetaPrime = std::atan2(centerPlantY, centerPlantX);

    this->setSpeed(200);

    this->rotate(this->robotPose.theta /*+ rotate*/ - thetaPrime);
    if (awaitRobotIdle() < 0) return;

    double robotPosForPotX = (centerPlantX * std::cos(theta) + centerPlantY * std::sin(theta)) + robotPosX;
    double robotPosForPotY = (-centerPlantX * std::sin(theta) + centerPlantY * std::cos(theta)) + robotPosY;


    this->transit(robotPosForPotX, robotPosForPotY, 130);
    if (awaitRobotIdle() < 0) return;

    this->closePince(pince);
    usleep(500'000);
    this->setSpeed(200);
    pinceState[pince] = TCPUtils::startWith(arucoTag.name(), "Purple_flower") ? PURPLE_FLOWER : WHITE_FLOWER;
    this->transportBras();
}

void TCPServer::askArduinoPos() {
    for (const auto & client : clients) {
        if (client.name == "arduino") {
            this->arduinoSocket = client.socket;
            break;
        }
    }

    if (this->arduinoSocket == -1) {
        return;
    }

    while (!this->_shouldStop) {
        this->sendToClient("strat;arduino;get pos;1\n", this->arduinoSocket);
        usleep(20'000);
    }
}

int TCPServer::awaitRobotIdle() {
    isRobotIdle = 0;
    int timeout = 0;
    // ReSharper disable once CppDFAConstantConditions
    // ReSharper disable once CppDFAEndlessLoop
    usleep(50'000);
    while (isRobotIdle < 2) {
        usleep(50'000);
        this->sendToClient("strat;arduino;get state;1\n", this->arduinoSocket);
        if (stopEmergency) {
            while (stopEmergency) {
                stopEmergency = false;
                usleep(300'000);
            }
            this->broadcastMessage(lastArduinoCommand);
            awaitRobotIdle();
        }
        if (gameStarted) {
            auto time = std::chrono::system_clock::now();
            if (time - gameStart > std::chrono::seconds(87)) {
                return -1;
            }
        }
        timeout++;
        if (timeout > 80) {
            this->broadcastMessage("strat;arduino;clear;1");
            return 1;
        }
    }
    return 0;
}

void TCPServer::handleArucoTag(const ArucoTag &tag) {
    if (!TCPUtils::contains(tag.name(), "flower")) {
        return;
    }

    auto rotArray = tag.rot();

    if (rotArray[2] > 0.3 && rotArray[2] < -0.3 && rotArray[0] > 3 && rotArray[0] < 2.5) {
        return;
    }

    for (auto& t : arucoTags) {
        if (tag.id() == t.id()) {
            auto [tPosX, tPosY] = t.pos();
            auto [tagPosX, tagPosY] = tag.pos();
            if (tagPosX > tPosX - 10 && tagPosX < tPosX + 10 && tagPosY > tPosY - 10 && tagPosY < tPosY + 10) {
                t.find();
                return;
            }
        }
    }

    this->arucoTags.push_back(tag);
}

std::optional<ArucoTag> TCPServer::getBiggestArucoTag(const float borneMinX, const float borneMaxX, const float borneMinY,
                                                      const float borneMaxY) {
    bool found = false;
    ArucoTag biggestTag = ArucoTag();
    for (const auto & tag : arucoTags) {
        if (tag.getNbFind() > biggestTag.getNbFind() && tag.pos()[0] > borneMinX && tag.pos()[0] < borneMaxX && tag.pos()[1] > borneMinY && tag.pos()[1] < borneMaxY) {
            biggestTag = tag;
            found = true;
        }
    }

    return found ? std::optional(biggestTag) : std::nullopt;
}


std::optional<ArucoTag> TCPServer::getMostCenteredArucoTag(const float borneMinX, const float borneMaxX, const float borneMinY, const float borneMaxY) {
    bool found = false;
    ArucoTag mostCenteredTag = ArucoTag();
    for (const auto & tag : arucoTags) {
        if (tag.getNbFind() < 2) continue;

        auto [tagX, tagY] = tag.pos();

        if (!found) {
            if (tagX > borneMinX && tagX < borneMaxX && tagY > borneMinY && tagY < borneMaxY) {
                mostCenteredTag = tag;
                found = true;
            }
        } else if (distanceToTag(tag) < distanceToTag(mostCenteredTag)) {
            if (tagX > borneMinX && tagX < borneMaxX && tagY > borneMinY && tagY < borneMaxY) {
                mostCenteredTag = tag;
            }
        }
    }

    return found ? std::optional(mostCenteredTag) : std::nullopt;
}

std::vector<PinceState> TCPServer::getNotFallenFlowers() const {
    std::vector<PinceState> res = {FLOWER, FLOWER, FLOWER};
    for (auto & tag : arucoTags) {
        if (TCPUtils::endWith(tag.name(), "flower") && tag.getNbFind() >= 1) {
            auto angle = tag.rot()[0];
            auto xPos = tag.pos()[0];
            auto yPos = tag.pos()[1];

            if (xPos > 700) continue;

            if (yPos > 70 && yPos < 200) {
                if (angle > 2.7f && angle < -2.f) {
                    res[2] = NONE;
                }
                else {
                    res[2] = (TCPUtils::contains(tag.name(), "White") ? WHITE_FLOWER : PURPLE_FLOWER);
                }
            }
            else if (yPos < -70 && yPos > -200) {
                if (angle > 2.7f && angle < -2.f) {
                    res[0] = NONE;
                }
                else {
                    res[0] = (TCPUtils::contains(tag.name(), "White") ? WHITE_FLOWER : PURPLE_FLOWER);
                }
            }
            else {
                if (angle > 2.7f && angle < -2.f) {
                    res[1] = NONE;
                }
                else {
                    res[1] = (TCPUtils::contains(tag.name(), "White") ? WHITE_FLOWER : PURPLE_FLOWER);
                }
            }
        }
    }
    return res;
}

void TCPServer::handleEmergency(int distance, double angle) {
    /*this->handleEmergencyFlag = true;

    this->broadcastMessage("strat;arduino;clear;2\n");

    // TODO handle here the emergency like wait for 2 second and then if emergency is again on, that means the robot of the other team do not move, if that go back otherwise continue
    usleep(2'000'000);
    // ReSharper disable once CppDFAConstantConditions
    while (this->stopEmergency) {
        // TODO here go back by twenty centimeter
        // ReSharper disable once CppDFAUnreachableCode
        this->broadcastMessage("strat;arduino;clear;3\n");

        this->stopEmergency = false;

        std::this_thread::sleep_for(std::chrono::seconds(1));*/

        /*double newAngle = this->robotPose.theta + angle;
        double newX = this->robotPose.pos.x + 200 * std::cos(newAngle);
        double newY = this->robotPose.pos.y + 200 * std::sin(newAngle);
        usleep(200'000);
        this->go(newX, newY);
        if (awaitRobotIdle() < 0) return;*/
    /*}
    this->broadcastMessage("strat;arduino;clear;4\n");

    try {
        this->gameThread.~thread();
    } catch (const std::exception& ex) {
        std::cout << ex.what() << std::endl;
    }

    this->gameStarted = false;

    this->gameThread = std::thread([this]() { this->startGame(); });

    this->gameThread.detach();

    this->handleEmergencyFlag = false;*/
}

void TCPServer::startTestAruco(const int pince) {
    this->arucoTags.clear();
    std::optional<ArucoTag> tag = std::nullopt;

    for (int i = 0; i < 5; i++) {
        this->broadcastMessage("strat;aruco;get aruco;1\n");
        usleep(220'000);
    }
    tag = getMostCenteredArucoTag(100, 800, -400, 400);

    int timeout = 0;
    while (!tag.has_value()) {
        this->broadcastMessage("strat;aruco;get aruco;1\n");
        usleep(220'000);
        tag = getMostCenteredArucoTag(100, 800, -400, 400);

        timeout++;
        if (timeout > 5) {
            break;
        }
    }

    if (tag.has_value()) {
        goToAruco(tag.value(), pince);
    }
}

void TCPServer::goEnd() {
    this->setSpeed(200);
    std::vector<std::array<int, 2>> checkponts;
    if (this->robotPose.pos.y > 1000) {
        if (team == BLUE) {
            checkponts.emplace_back(std::array{500, 1700});
        } else if (team == YELLOW) {
            checkponts.emplace_back(std::array{2500, 1700});
        }
    }

    for (const auto& checkpoint : checkponts) {
        this->go(checkpoint);
        if (awaitRobotIdle() < 0) return;
    }

    this->go(this->endRobotPose.pos.x, this->endRobotPose.pos.y);
    if (awaitRobotIdle() < 0) return;
    this->setSpeed(180);
    this->rotate(this->endRobotPose.theta);
    if (awaitRobotIdle() < 0) return;

    this->baisserBras();

    for (int i = 0 ; i < 3; i++) {
        this->openPince(i);
        usleep(50'000);
    }
    this->sendPoint(10);

    this->broadcastMessage("strat;all;end;1");
}

void TCPServer::findAndGoFlower(const StratPattern sp) {
    this->setSpeed(200);
    if (team == BLUE) {
        if (sp == TAKE_FLOWER_TOP) {
            this->go(500, 700);
            if (awaitRobotIdle() < 0) return;

            this->rotate(0);
            if (awaitRobotIdle() < 0) return;
        }
        else if (sp == TAKE_FLOWER_BOTTOM) {
            this->go(500, 1300);
            if (awaitRobotIdle() < 0) return;

            this->rotate(0);
            if (awaitRobotIdle() < 0) return;
        } else {
            return;
        }
    } else if (team == YELLOW) {
        if (sp == TAKE_FLOWER_TOP) {
            this->go(1500,  700);
            if (awaitRobotIdle() < 0) return;

            this->rotate(-PI);
            if (awaitRobotIdle() < 0) return;
        }
        else if (sp == TAKE_FLOWER_BOTTOM) {
            this->go(1500, 1300);
            if (awaitRobotIdle() < 0) return;

            this->rotate(-PI);
            if (awaitRobotIdle() < 0) return;
        } else {
            return;
        }
    } else {
        return;
    }

    this->arucoTags.clear();
    std::optional<ArucoTag> tag = std::nullopt;

    for (int i = 0; i < 5; i++) {
        this->broadcastMessage("strat;aruco;get aruco;1\n");
        usleep(110'000);
    }
    tag = getMostCenteredArucoTag(300, 700, -200, 200);

    int timeout = 0;
    while (!tag.has_value()) {
        this->broadcastMessage("strat;aruco;get aruco;1\n");
        usleep(110'000);
        tag = getMostCenteredArucoTag(300, 700, -200, 200);

        timeout++;
        if (timeout > 3) {
            break;
        }
    }

    if (tag.has_value()) {
        /*if (pinceState[1] == NONE) {
            goToAruco(tag.value(), 1);
        } else if (pinceState[2] == NONE) {
            goToAruco(tag.value(), 2);
        } else if (pinceState[0] == NONE) {
            goToAruco(tag.value(), 0);
        }*/
        if (pinceState[0] == NONE) {
            goToAruco(tag.value(), 0);
        } else if (pinceState[2] == NONE) {
            goToAruco(tag.value(), 2);
        } else if (pinceState[1] == NONE) {
            goToAruco(tag.value(), 1);
        }
    }
}

void TCPServer::dropPurpleFlowers() {
    std::vector<int> pinceHavePurpleFlower;

    pinceHavePurpleFlower.reserve(3);

    for (int i = 0; i < 3; i++) {
        switch (pinceState[i]) {
            case PURPLE_FLOWER:
                pinceHavePurpleFlower.push_back(i);
                break;
            default:
                break;
        }
    }

    std::array<int, 2> purpleDropPosition{};
    if (team == BLUE) {
        purpleDropPosition = {200, 400};
    } else if (team == YELLOW) {
        purpleDropPosition = {1800, 400};
    }

    this->setSpeed(200);

    if (!pinceHavePurpleFlower.empty()) {
        this->go(purpleDropPosition);
        if (awaitRobotIdle() < 0) return;

        this->setSpeed(150);

        this->rotate(PI / 2);
        if (awaitRobotIdle() < 0) return;

        this->baisserBras();

        for (const auto & toDrop : pinceHavePurpleFlower) {
            this->openPince(toDrop);
            usleep(200'000);

            this->go(this->robotPose.pos.x, this->robotPose.pos.y - 150);
            if (awaitRobotIdle() < 0) return;

            this->go(this->robotPose.pos.x, this->robotPose.pos.y + 150);
            if (awaitRobotIdle() < 0) return;

            pinceState[toDrop] = NONE;
            this->closePince(toDrop);
            usleep(200'000);

            this->sendPoint(3);
        }
    }

    this->transportBras();

    this->setSpeed(200);
}

void TCPServer::dropWhiteFlowers(const StratPattern sp) {
    std::vector<int> pinceHaveWhiteFlower;

    pinceHaveWhiteFlower.reserve(3);

    for (int i = 0; i < 3; i++) {
        switch (pinceState[i]) {
            case WHITE_FLOWER:
                pinceHaveWhiteFlower.push_back(i);
            break;
            default:
                break;
        }
    }

    if (pinceHaveWhiteFlower.empty()) {
        return;
    }

    this->setSpeed(200);

    std::array<int, 2> whiteDropSetup{};
    std::array<int, 2> whiteDropPosition{};
    double angle = PI / 2;
    if (team == BLUE) {
        if (sp == DROP_WHITE_FLOWER_J1) {
            whiteDropSetup = std::array{762, 300};
            whiteDropPosition = std::array{762, 0};
            angle = PI / 2;
        } else if (sp == DROP_WHITE_FLOWER_J2) {
            whiteDropSetup = std::array{300, 612};
            whiteDropPosition = std::array{0, 612};
            angle = -PI;
        }
    } else if (team == YELLOW) {
        if (sp == DROP_WHITE_FLOWER_J1) {
            whiteDropSetup = std::array{2237, 300};
            whiteDropPosition = std::array{2237, 0};
            angle = PI / 2;
        } else if (sp == DROP_WHITE_FLOWER_J2) {
            whiteDropSetup = std::array{1700, 612};
            whiteDropPosition = std::array{0, 612};
            angle = 0;

        }
    }

    this->setSpeed(200);

    this->go(whiteDropSetup);
    if (awaitRobotIdle() < 0) return;


    this->rotate(angle);
    if (awaitRobotIdle() < 0) return;

    this->leverBras();
    usleep(500'000);

    this->setSpeed(130);

    this->go(whiteDropPosition);
    usleep(2'000'000);

    for (int i = 0; i < 3; i++) {
        if (pinceState[i] == WHITE_FLOWER) {
            this->openPince(i);
            usleep(1'000'000);

            pinceState[i] = NONE;
            this->closePince(i);
            usleep(100'000);

            this->sendPoint(4);
        }
    }

    for (int i = 0; i < 3; i++) {
        this->middlePince(i);
    }
    usleep(1'000'000);

    this->setSpeed(200);

    this->go(whiteDropSetup);
    if (awaitRobotIdle() < 0) return;

    for (int i = 0; i < 3; i++) {
        this->closePince(i);
    }

    this->transportBras();
}

void TCPServer::goAndTurnSolarPanel(const StratPattern sp) {
    this->setSpeed(170);
    if (team == BLUE) {
        switch (sp) {
            case TURN_SOLAR_PANNEL_1:
                this->go(250, 1800);
                if (awaitRobotIdle() < 0) return;

                this->rotate(0);
                if (awaitRobotIdle() < 0) return;

                this->checkPanneau(7);
                usleep(300'000);
                this->uncheckPanneau(7);
                break;
            case TURN_SOLAR_PANNEL_2:
                this->go(460, 1800);
                if (awaitRobotIdle() < 0) return;

                this->rotate(0);
                if (awaitRobotIdle() < 0) return;

                this->checkPanneau(7);
                usleep(300'000);
                this->uncheckPanneau(7);
                break;
            case TURN_SOLAR_PANNEL_3:
                this->go(690, 1800);
                if (awaitRobotIdle() < 0) return;

                this->rotate(0);
                if (awaitRobotIdle() < 0) return;

                this->checkPanneau(7);
                usleep(300'000);
                this->uncheckPanneau(7);
                break;
            default:
                break;
        }
    } else if (team == YELLOW) {
        switch (sp) {
            case TURN_SOLAR_PANNEL_1:
                this->go(2750, 1800);
                if (awaitRobotIdle() < 0) return;

                this->rotate(PI);
                if (awaitRobotIdle() < 0) return;

                this->checkPanneau(6);
                usleep(300'000);
                this->uncheckPanneau(6);
                break;
            case TURN_SOLAR_PANNEL_2:
                this->go(2540, 1800);
                if (awaitRobotIdle() < 0) return;

                this->rotate(PI);
                if (awaitRobotIdle() < 0) return;

                this->checkPanneau(6);
                usleep(300'000);
                this->uncheckPanneau(6);
                break;
            case TURN_SOLAR_PANNEL_3:
                this->go(2310, 1800);
                if (awaitRobotIdle() < 0) return;

                this->rotate(PI);
                if (awaitRobotIdle() < 0) return;

                this->checkPanneau(6);
                usleep(300'000);
                this->uncheckPanneau(6);
                break;
            default:
                break;
        }
    }

    this->sendPoint(5);
}

void TCPServer::dropJardiniereFlowers(const StratPattern sp) {

    std::array<int, 2> whiteDropSetup{};
    std::array<int, 2> whiteDropPosition{};
    double angle = PI / 2;
    if (team == BLUE) {
        if (sp == DROP_FLOWER_J1) {
            whiteDropSetup = std::array{755, 300};
            whiteDropPosition = std::array{755, 0};
            angle = PI / 2;
        } else if (sp == DROP_FLOWER_J2) {
            whiteDropSetup = std::array{300, 600};
            whiteDropPosition = std::array{0, 600};
            angle = -PI;
        }
    } else if (team == YELLOW) {
        if (sp == DROP_FLOWER_J1) {
            whiteDropSetup = std::array{2230, 300};
            whiteDropPosition = std::array{2230, 0};
            angle = PI / 2;
        } else if (sp == DROP_FLOWER_J2) {
            whiteDropSetup = std::array{2700, 600};
            whiteDropPosition = std::array{3000, 600};
            angle = 0;
        }
    }

    this->setMaxSpeed();

    this->transit(whiteDropSetup, 170);
    if (awaitRobotIdle() < 0) return;

    this->rotate(angle);
    if (awaitRobotIdle() < 0) return;

    this->leverBras();

    this->setSpeed(130);

    this->go(whiteDropPosition);
    usleep(2'000'000);

    if (pinceState[0] != NONE) {
        this->fullyOpenPince(0);
        pinceState[0] = NONE;
    }
    if (pinceState[2] != NONE) {
        this->fullyOpenPince(2);
        pinceState[2] = NONE;
    }

    usleep(500'000);

    this->closePince(0);
    this->closePince(2);

    usleep(100'000);

    if (pinceState[1] != NONE) {
        this->fullyOpenPince(1);
        pinceState[1] = NONE;
    }

    this->sendPoint(3+1);
    this->sendPoint(3+1);

    usleep(500'000);

    this->openPince(0);
    this->openPince(1);
    this->openPince(2);

    this->setMaxSpeed();

    this->go(whiteDropSetup);
    if (awaitRobotIdle() < 0) return;

    this->transportBras();
}

void TCPServer::dropBaseFlowers(StratPattern sp) {
    std::array<int, 2> dropPosition{};
    double angle;
    float distance;

    if (team == BLUE) {
        if (sp == DROP_FLOWER_BASE_1) {
            dropPosition = {300, 400};
            angle = PI / 2;
            distance = 150;
        }
        else if (sp == DROP_FLOWER_BASE_2) {
            dropPosition = {300, 1600};
            angle = -PI / 2;
            distance = -150;
        }
        else {
            return;
        }
    }
    else if (team == YELLOW) {
        if (sp == DROP_FLOWER_BASE_1) {
            dropPosition = {2700, 400};
            angle = PI / 2;
            distance = 150;
        }
        else if (sp == DROP_FLOWER_BASE_2) {
            dropPosition = {2700, 1600};
            angle = -PI / 2;
            distance = -150;
        } else {
            return;
        }
    }
    else {
        return;
    }

    this->setMaxSpeed();

    this->go(dropPosition);
    if (awaitRobotIdle() < 0) return;

    this->setSpeed(170);

    this->rotate(angle);
    if (awaitRobotIdle() < 0) return;

    this->setMaxSpeed();

    this->baisserBras();

    for (int i = 0; i < 3; i++) {
        this->openPince(i);
    }

    this->go(this->robotPose.pos.x, this->robotPose.pos.y - distance);
    if (awaitRobotIdle() < 0) return;

    this->go(this->robotPose.pos.x, this->robotPose.pos.y + distance);
    if (awaitRobotIdle() < 0) return;

    bool detectedPurple = false;

    for (int i = 0; i < 3; i++) {
        if (pinceState[i] == PURPLE_FLOWER) {
            detectedPurple = true;
            this->sendPoint(3);
        }

        pinceState[i] = NONE;
        this->closePince(i);
    }

    if (!detectedPurple) {
        this->sendPoint(3);
    }

    this->transportBras();

    this->setMaxSpeed();
}

void TCPServer::go3Plants(const StratPattern sp) {
    std::array<int, 2> plantPosition{};
    double direction;

    double angle;
    if (team == BLUE) {
        angle = 0;
        direction = 1;
        if (sp == TAKE_3_PLANT_TOP_1) {
            plantPosition = {900, 700};
        }
        else if (sp == TAKE_3_PLANT_TOP_2) {
            plantPosition = {1100, 700};
        }
        else if (sp == TAKE_3_PLANT_BOTTOM_1) {
            plantPosition = {900, 1300};
        }
        else if (sp == TAKE_3_PLANT_BOTTOM_2) {
            plantPosition = {1100, 1300};
        }
        else {
            return;
        }
    }
    else if (team == YELLOW) {
        angle = -PI;
        direction = -1;

        if (sp == TAKE_3_PLANT_TOP_1) {
            plantPosition = {2100, 700};
        }
        else if (sp == TAKE_3_PLANT_TOP_2) {
            plantPosition = {1900, 700};
        }
        else if (sp == TAKE_3_PLANT_BOTTOM_1) {
            plantPosition = {2100, 1300};
        }
        else if (sp == TAKE_3_PLANT_BOTTOM_2) {
            plantPosition = {1900, 1300};
        }
        else {
            return;
        }
    }
    else {
        return;
    }

    this->setMaxSpeed();

    this->transit(plantPosition[0]-(600*direction), plantPosition[1], 170);
    // this->go(plantPosition[0]-500, plantPosition[1]);
    if (awaitRobotIdle() < 0) return;

    this->baisserBras();

    this->setMaxSpeed();

    this->rotate(angle);
    if (awaitRobotIdle() < 0) return;

    this->transit(plantPosition[0]-(400*direction), plantPosition[1], 150);
    if (awaitRobotIdle() < 0) return;

    this->rotate(angle);
    if (awaitRobotIdle() < 0) return;

    this->setMaxSpeed();

    this->arucoTags.clear();
    for (int i = 0; i < 5; i++) {
        this->broadcastMessage("strat;aruco;get aruco;1\n");
        usleep(110'000);
    }

    std::vector<PinceState> pinceCanTakeFLower = getNotFallenFlowers();

    for (int i = 0; i < 3; i++) {
        if (pinceCanTakeFLower[i] != NONE) {
            this->openPince(i);
        }
    }
    usleep(200'000);

    this->transit(plantPosition[0], this->robotPose.pos.y, 130);
    if (awaitRobotIdle() < 0) return;

    this->setMaxSpeed();

    usleep(500'000);

    for (int i = 0; i < 3; i++) {
        this->closePince(i);
        pinceState[i] = pinceCanTakeFLower[i];
    }
    usleep(500'000);

    this->rotate(angle);
    if (awaitRobotIdle() < 0) return;

    this->setSpeed(160);

    this->go(this->robotPose.pos.x - (200 * direction), this->robotPose.pos.y);
    if (awaitRobotIdle() < 0) return;

    for (int i = 0; i < 3; i++) {
        this->openPince(i);
    }
    usleep(200'000);

    this->setSpeed(150);
    this->go(this->robotPose.pos.x + (75 * direction), this->robotPose.pos.y);
    if (awaitRobotIdle() < 0) return;

    for (int i = 0; i < 3; i++) {
        this->closePince(i);
    }
    usleep(500'000);

    this->transportBras();
}

void TCPServer::removePot(StratPattern sp) {
    this->setMaxSpeed();
    if (team == BLUE) {
        if (sp == REMOVE_POT_J2) {
            this->transit(250, 1100, 150);
            if (awaitRobotIdle() < 0) return;
            this->setMaxSpeed();
            this->go(210, 900);
            if (awaitRobotIdle() < 0) return;
            this->go(200, 400);
            if (awaitRobotIdle() < 0) return;
            this->go(250, 650);
            if (awaitRobotIdle() < 0) return;
        }
    } else if (team == YELLOW) {
        if (sp == REMOVE_POT_J2) {
            this->transit(2750, 1100, 150);
            if (awaitRobotIdle() < 0) return;
            this->setMaxSpeed();
            this->go(2780, 900);
            if (awaitRobotIdle() < 0) return;
            this->go(2780, 400);
            if (awaitRobotIdle() < 0) return;
            this->go(2750, 650);
            if (awaitRobotIdle() < 0) return;
        }
    }
}

void TCPServer::getLidarPos() {

    this->lidarGetPosTimeout = 0;

    this->broadcastMessage("strat;arduino;clear;1\n");

    usleep(1'000'000);

    this->setPosition(this->robotPose, lidarSocket);

    usleep(100'000);

    awaitForLidar = true;

    this->askLidarPosition();

    int timeout = 0;

    // ReSharper disable once CppDFAConstantConditions
    // ReSharper disable once CppDFAEndlessLoop
    while (awaitForLidar) {
        usleep(50'000);
        timeout++;
        if (timeout > 100) {
            break;
        }
    }

    // ReSharper disable once CppDFAUnreachableCode
    std::cout << lidarCalculatePos.pos.x << " " << lidarCalculatePos.pos.y << " " << lidarCalculatePos.theta << std::endl;

}

void TCPServer::checkpoint(const StratPattern sp) {
    this->setMaxSpeed();
    if (team == BLUE) {
        switch (sp) {
            case CHECKPOINT_MIDDLE:
                this->go(500, 1500);
                if (awaitRobotIdle() < 0) return;
                break;
            case CHECKPOINT_TRANSITION_SOLAR_PANEL_FLOWER:
                this->go(800, 1800);
                if (awaitRobotIdle() < 0) return;
                this->go(500, 1700);
                usleep(500'000);
                break;
            default:
                break;
        }
    } else if (team == YELLOW) {
        switch (sp) {
            case CHECKPOINT_MIDDLE:
                this->go(2500, 1500);
                if (awaitRobotIdle() < 0) return;
                break;
            case CHECKPOINT_TRANSITION_SOLAR_PANEL_FLOWER:
                this->go(2200, 1800);
                if (awaitRobotIdle() < 0) return;
                this->go(2500, 1700);
                usleep(500'000);
                break;
            default:
                break;
        }
    }
}

template<class X, class Y>
void TCPServer::go(X x, Y y) {
    lastArduinoCommand = "strat;arduino;go;" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "\n";
    this->broadcastMessage("strat;arduino;go;" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "\n");
}

template<class X>
void TCPServer::go(std::array<X, 2> data) {
    lastArduinoCommand = "strat;arduino;go;" + std::to_string(static_cast<int>(data[0])) + "," + std::to_string(static_cast<int>(data[1])) + "\n";
    this->broadcastMessage("strat;arduino;go;" + std::to_string(static_cast<int>(data[0])) + "," + std::to_string(static_cast<int>(data[1])) + "\n");
}

template<class X>
void TCPServer::rotate(X angle) {
    lastArduinoCommand = "strat;arduino;angle;" + std::to_string(static_cast<int>(angle * 100)) + "\n";
    this->broadcastMessage("strat;arduino;angle;" + std::to_string(static_cast<int>(angle * 100)) + "\n");
}

void TCPServer::setSpeed(const int speed) {
    this->broadcastMessage("strat;arduino;speed;" + std::to_string(speed) + "\n");
    this->speed = speed;
}

void TCPServer::setMaxSpeed() {
    this->setSpeed(MAX_SPEED);
}

void TCPServer::setMinSpeed() {
    this->setSpeed(MIN_SPEED);
}

template<class X, class Y>
void TCPServer::transit(X x, Y y, const int endSpeed) {
    lastArduinoCommand = "strat;arduino;transit;" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "," + std::to_string(endSpeed) + "\n";
    this->broadcastMessage("strat;arduino;transit;" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "," + std::to_string(endSpeed) + "\n");
}

template<class X>
void TCPServer::transit(std::array<X, 2> data, const int endSpeed) {
    lastArduinoCommand = "strat;arduino;transit;" + std::to_string(static_cast<int>(data[0])) + "," + std::to_string(static_cast<int>(data[1])) + "," + std::to_string(endSpeed) + "\n";
    this->broadcastMessage("strat;arduino;transit;" + std::to_string(static_cast<int>(data[0])) + "," + std::to_string(static_cast<int>(data[1])) + "," + std::to_string(endSpeed) + "\n");
}

template<class X, class Y, class Z>
void TCPServer::setPosition(X x, Y y, Z theta, const int clientSocket) {
    if (clientSocket == -1) {
        this->broadcastMessage("strat;all;set pos;" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "," + std::to_string(static_cast<int>(theta * 100)) + "\n");
    } else {
        this->sendToClient("strat;all;set pos;" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "," + std::to_string(static_cast<int>(theta * 100)) + "\n", clientSocket);
    }
}

template<class X>
void TCPServer::setPosition(std::array<X, 3> data, const int clientSocket) {
    if (clientSocket == -1) {
        this->broadcastMessage("strat;all;set pos;" + std::to_string(static_cast<int>(data[0])) + "," + std::to_string(static_cast<int>(data[1])) + "," + std::to_string(static_cast<int>(data[2] * 100)) + "\n");
    } else {
        this->sendToClient("strat;all;set pos;" + std::to_string(static_cast<int>(data[0])) + "," + std::to_string(static_cast<int>(data[1])) + "," + std::to_string(static_cast<int>(data[2] * 100)) + "\n", clientSocket);
    }
}

void TCPServer::setPosition(const Position pos, const int clientSocket) {
    if (clientSocket == -1) {
        this->broadcastMessage("strat;all;set pos;" + std::to_string(static_cast<int>(pos.pos.x)) + "," + std::to_string(static_cast<int>(pos.pos.y)) + "," + std::to_string(static_cast<int>(pos.theta * 100)) + "\n");
    } else {
        this->sendToClient("strat;lidar;set pos;" + std::to_string(static_cast<int>(pos.pos.x)) + "," + std::to_string(static_cast<int>(pos.pos.y)) + "," + std::to_string(static_cast<int>(pos.theta * 100)) + "\n", clientSocket);
    }
}

template<class X, class Y, class Z>
void TCPServer::setPosition(X x, Y y, Z theta, const std::string &toSend) {
    this->broadcastMessage("strat;" + toSend + ";set pos;" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "," + std::to_string(static_cast<int>(theta * 100)) + "\n");
}

template<class X>
void TCPServer::setPosition(std::array<X, 3> data, const std::string &toSend) {
    this->broadcastMessage("strat;" + toSend + ";set pos;" + std::to_string(static_cast<int>(data[0])) + "," + std::to_string(static_cast<int>(data[1])) + "," + std::to_string(static_cast<int>(data[2] * 100)) + "\n");
}

void TCPServer::setPosition(const Position pos, const std::string &toSend) {
    this->broadcastMessage("strat;" + toSend + ";set pos;" + std::to_string(static_cast<int>(pos.pos.x)) + "," + std::to_string(static_cast<int>(pos.pos.y)) + "," + std::to_string(static_cast<int>(pos.theta * 100)) + "\n");
}

void TCPServer::baisserBras() {
    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
}

void TCPServer::transportBras() {
    this->broadcastMessage("strat;servo_moteur;transport bras;1\n");
}

void TCPServer::leverBras() {
    this->broadcastMessage("strat;servo_moteur;lever bras;1\n");
}

void TCPServer::openPince(int pince) {
    this->broadcastMessage("strat;servo_moteur;ouvrir pince;" + std::to_string(pince) + "\n");
}

void TCPServer::fullyOpenPince(int pince) {
    this->broadcastMessage("strat;servo_moteur;ouvrir total pince;" + std::to_string(pince) + "\n");
}

void TCPServer::middlePince(int pince) {
    this->broadcastMessage("strat;servo_moteur;middle pince;" + std::to_string(pince) + "\n");
}

void TCPServer::closePince(int pince) {
    this->broadcastMessage("strat;servo_moteur;fermer pince;" + std::to_string(pince) + "\n");
}

void TCPServer::checkPanneau(int servo_moteur) {
    this->broadcastMessage("strat;servo_moteur;check panneau;" + std::to_string(servo_moteur) + "\n");
}

void TCPServer::uncheckPanneau(int servo_moteur) {
    this->broadcastMessage("strat;servo_moteur;uncheck panneau;" + std::to_string(servo_moteur) + "\n");
}

void TCPServer::askLidarPosition() {
    this->broadcastMessage("start;lidar;get pos;1\n");
}

void TCPServer::sendPoint(int point) {
    this->broadcastMessage("strat;ihm;add point;" + std::to_string(point) + "\n");
}

void TCPServer::setTeam(Team team) {
    this->team = team;
    this->broadcastMessage("strat;all;set team;" + std::to_string(team) + "\n");
}
