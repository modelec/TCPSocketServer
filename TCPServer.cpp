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
    if (tokens[2] == "stop proximity") {
        if (!gameStarted) return;

        this->stopEmergency = true;
        this->gameThread.~thread();
        this->broadcastMessage("strat;arduino;clear;1");

        std::vector<std::string> args = TCPUtils::split(tokens[3], ",");

        if (!handleEmergencyFlag) {
            std::thread([this, args]() { this->handleEmergency(std::stoi(args[0]), std::stod(args[1]) / 100); }).detach();
        }
    }
    else if (tokens[1] != "strat")
    {
        this->broadcastMessage(message, clientSocket);
    }
    // EMERGENCY
    else if (tokens[0] == "tirette" && tokens[2] == "set state")
    {
        this->broadcastMessage(message, clientSocket);
    }
    else if (tokens[2] == "ready")
    {
        for (ClientTCP& client : clients)
        {
            if (client.name == tokens[0])
            {
                client.isReady = true;
                client.socket = clientSocket;
                if (client.name == "lidar") {
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
        this->lidarCalculatePos = {std::stof(args[0]), std::stof(args[1]), std::stof(args[2]) / 100};
        this->setPosition(this->lidarCalculatePos);
        usleep(100'000);
        this->setPosition(this->lidarCalculatePos);
        awaitForLidar = false;
    }

    else if (tokens[0] == "ihm") {
        if (tokens[2] == "spawn") {
            int spawnPointNb = std::stoi(tokens[3]);
            float spawnPoint[3];
            float finishPoint[3];

            switch (spawnPointNb) {
                case 3:
                    this->team = BLUE;
                    spawnPoint[0] = 250;
                    spawnPoint[1] = 1790;
                    spawnPoint[2] = 0;

                    // For test
                    /*finishPoint[0] = 400;
                    finishPoint[1] = 1790;
                    finishPoint[2] = 0;*/

                    finishPoint[0] = 400;
                    finishPoint[1] = 500;
                    finishPoint[2] = PI / 2;
                    break;
                case 6:
                    this->team = YELLOW;
                    spawnPoint[0] = 1750;
                    spawnPoint[1] = 1790;
                    spawnPoint[2] = PI;

                    finishPoint[0] = 2600;
                    finishPoint[1] = 500;
                    finishPoint[2] = PI / 2;
                    break;

                default:
                    this->team = TEST;
                    spawnPoint[0] = 1200;
                    spawnPoint[1] = 1800;
                    spawnPoint[2] = 1.57;

                    finishPoint[0] = 1200;
                    finishPoint[1] = 1800;
                    finishPoint[2] = 1.57;
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
                usleep(200'000);
            }
        }
        else if (tokens[1] == "strat" && tokens[2] == "start")
        {
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
            this->setPosition(this->robotPose, lidarSocket);
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
    this->gameThread.join();

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
    for (int i = whereAmI; i < stratPatterns.size(); i++) {

        /*auto time = std::chrono::system_clock::now();
        if (time - gameStart > std::chrono::seconds(82)) {
            this->goEnd();
            return;
        }*/

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
            case CHECKPOINT_BOTTOM_TO_TOP:
                checkpoint(CHECKPOINT_BOTTOM_TO_TOP);
                break;
            case CHECKPOINT_TOP_TO_BOTTOM:
                checkpoint(CHECKPOINT_TOP_TO_BOTTOM);
                break;
            case CHECKPOINT_TRANSITION_SOLAR_PANEL_FLOWER:
                checkpoint(CHECKPOINT_TRANSITION_SOLAR_PANEL_FLOWER);
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
    awaitRobotIdle();

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
    awaitRobotIdle();

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
    awaitRobotIdle();

    this->broadcastMessage("strat;arduino;angle;157\n");
    awaitRobotIdle();

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
    awaitRobotIdle();

    toSend = "strat;arduino;angle;" + std::to_string(static_cast<int>(this->endRobotPose.theta * 100)) + "\n";
    this->broadcastMessage(toSend);
    awaitRobotIdle();

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
            decalage = 60;
            break;
        case 1:
            decalage = 0;
            break;
        case 2:
            decalage = -60;
            break;
        default:
            decalage = 0;
            break;
    }

    this->baisserBras();
    this->openPince(pince);

    auto [xPrime, yPrime] = arucoTag.pos();
    double roll = arucoTag.rot()[1];

    auto centerPlantX = (20 * std::cos(roll)) + xPrime - 20;
    auto centerPlantY = (-20 * std::sin(roll)) + yPrime + decalage;

    double thetaPrime = std::atan2(centerPlantY, centerPlantX);

    this->setSpeed(200);

    this->rotate(this->robotPose.theta /*+ rotate*/ - thetaPrime);
    awaitRobotIdle();

    double robotPosForPotX = (centerPlantX * std::cos(theta) + centerPlantY * std::sin(theta)) + robotPosX;
    double robotPosForPotY = (-centerPlantX * std::sin(theta) + centerPlantY * std::cos(theta)) + robotPosY;


    this->transit(robotPosForPotX, robotPosForPotY, 130);
    awaitRobotIdle();

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
        usleep(200'000);
    }
}

void TCPServer::awaitRobotIdle() {
    isRobotIdle = 0;
    int timeout = 0;
    // ReSharper disable once CppDFAConstantConditions
    // ReSharper disable once CppDFAEndlessLoop
    while (isRobotIdle < 3) {
        usleep(50'000);
        this->sendToClient("strat;arduino;get state;1\n", this->arduinoSocket);
        timeout++;
        /*if (timeout > 30) {
            this->broadcastMessage("strat;arduino;clear;1");
            break;
        }*/
    }
}

void TCPServer::handleArucoTag(ArucoTag &tag) {
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

std::optional<ArucoTag> TCPServer::getBiggestArucoTag(float borneMinX, float borneMaxX, float borneMinY,
                                                      float borneMaxY) {
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


std::optional<ArucoTag> TCPServer::getMostCenteredArucoTag(float borneMinX, float borneMaxX, float borneMinY, float borneMaxY) {
    bool found = false;
    ArucoTag mostCenteredTag = ArucoTag();
    for (const auto & tag : arucoTags) {
        // if (tag.getNbFind() < 2) continue;

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

void TCPServer::handleEmergency(int distance, double angle) {
    this->handleEmergencyFlag = true;

    // TODO handle here the emergency like wait for 2 second and then if emergency is again on, that means the robot of the other team do not move, if that go back otherwise continue
    usleep(2'000'000);
    this->stopEmergency = false;
    usleep(500'000);
    if (this->stopEmergency) {
        // TODO here go back by twenty centimeter
        usleep(200'000);
    }

    this->handleEmergencyFlag = false;

    this->gameThread.~thread();

    this->gameStarted = false;

    this->gameThread = std::thread([this]() { this->startGame(); });

    this->gameThread.detach();
}

void TCPServer::startTestAruco(int pince) {
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
        awaitRobotIdle();
    }

    this->go(this->endRobotPose.pos.x, this->endRobotPose.pos.y);
    awaitRobotIdle();
    this->setSpeed(180);
    this->rotate(this->endRobotPose.theta);
    awaitRobotIdle();

    this->broadcastMessage("strat;all;end;1");
}

void TCPServer::findAndGoFlower(const StratPattern sp) {
    this->setSpeed(200);
    if (team == BLUE) {
        if (sp == TAKE_FLOWER_TOP) {
            this->go(500, 700);
            awaitRobotIdle();

            this->rotate(0);
            awaitRobotIdle();
        }
        else if (sp == TAKE_FLOWER_BOTTOM) {
            this->go(500, 1300);
            awaitRobotIdle();

            this->rotate(0);
            awaitRobotIdle();
        } else {
            return;
        }
    } else if (team == YELLOW) {
        if (sp == TAKE_FLOWER_TOP) {
            this->go(1500,  700);
            awaitRobotIdle();

            this->rotate(-PI);
            awaitRobotIdle();
        }
        else if (sp == TAKE_FLOWER_BOTTOM) {
            this->go(1500, 1300);
            awaitRobotIdle();

            this->rotate(-PI);
            awaitRobotIdle();
        } else {
            return;
        }
    } else {
        return;
    }

    this->arucoTags.clear();
    std::optional<ArucoTag> tag = std::nullopt;

    for (int i = 0; i < 4; i++) {
        this->broadcastMessage("strat;aruco;get aruco;1\n");
        usleep(110'000);
    }
    tag = getMostCenteredArucoTag(300, 1000, -200, 200);

    int timeout = 0;
    while (!tag.has_value()) {
        this->broadcastMessage("strat;aruco;get aruco;1\n");
        usleep(110'000);
        tag = getMostCenteredArucoTag(300, 1000, -200, 200);

        timeout++;
        if (timeout > 3) {
            break;
        }
    }

    if (tag.has_value()) {
        if (pinceState[1] == NONE) {
            goToAruco(tag.value(), 1);
        } else if (pinceState[2] == NONE) {
            goToAruco(tag.value(), 2);
        } else if (pinceState[0] == NONE) {
            goToAruco(tag.value(), 0);
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
            case WHITE_FLOWER:
            case NONE:
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
        awaitRobotIdle();

        this->setSpeed(150);

        this->rotate(PI / 2);
        awaitRobotIdle();

        this->baisserBras();

        for (auto & toDrop : pinceHavePurpleFlower) {
            this->openPince(toDrop);
            usleep(200'000);

            this->go(this->robotPose.pos.x, this->robotPose.pos.y - 150);
            awaitRobotIdle();

            this->go(this->robotPose.pos.x, this->robotPose.pos.y + 150);
            awaitRobotIdle();

            pinceState[toDrop] = NONE;
            this->closePince(toDrop);
            usleep(200'000);
        }
    }

    this->transportBras();

    this->setSpeed(200);
}

void TCPServer::dropWhiteFlowers(StratPattern sp) {
    std::vector<int> pinceHaveWhiteFlower;

    pinceHaveWhiteFlower.reserve(3);

    for (int i = 0; i < 3; i++) {
        switch (pinceState[i]) {
            case WHITE_FLOWER:
                pinceHaveWhiteFlower.push_back(i);
            break;
            case PURPLE_FLOWER:
            case NONE:
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
            this->go(200, 400);
            awaitRobotIdle();
            this->go(200, 1000);
            awaitRobotIdle();
            this->go(220, 650);
            awaitRobotIdle();

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
            this->go(2800, 400);
            awaitRobotIdle();
            this->go(2800, 1000);
            awaitRobotIdle();
            this->go(1780, 650);
            awaitRobotIdle();

            whiteDropSetup = std::array{1700, 612};
            whiteDropPosition = std::array{0, 612};
            angle = 0;

        }
    }

    this->setSpeed(200);

    this->go(whiteDropSetup);
    awaitRobotIdle();


    this->rotate(angle);
    awaitRobotIdle();

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
        }
    }

    for (int i = 0; i < 3; i++) {
        this->middlePince(i);
    }
    usleep(1'000'000);

    this->setSpeed(200);

    this->go(whiteDropSetup);
    awaitRobotIdle();

    for (int i = 0; i < 3; i++) {
        this->closePince(i);
    }

    this->transportBras();
}

void TCPServer::goAndTurnSolarPanel(StratPattern sp) {
    int previousSpeed = this->speed;
    this->setSpeed(200);
    if (team == BLUE) {
        switch (sp) {
            case TURN_SOLAR_PANNEL_1:
                this->go(250, 1790);
                awaitRobotIdle();

                this->rotate(0);
                awaitRobotIdle();

                this->checkPanneau(7);
                usleep(150'000);
                this->uncheckPanneau(7);
                break;
            case TURN_SOLAR_PANNEL_2:
                this->go(475, 1790);
                awaitRobotIdle();

                this->rotate(0);
                awaitRobotIdle();

                this->checkPanneau(7);
                usleep(150'000);
                this->uncheckPanneau(7);
                break;
            case TURN_SOLAR_PANNEL_3:
                this->go(700, 1790);
                awaitRobotIdle();

                this->rotate(0);
                awaitRobotIdle();

                this->checkPanneau(7);
                usleep(150'000);
                this->uncheckPanneau(7);
                break;
            default:
                break;
        }
    } else if (team == YELLOW) {
        switch (sp) {
            case TURN_SOLAR_PANNEL_1:
                this->go(2750, 1790);
                awaitRobotIdle();

                this->rotate(PI);
                awaitRobotIdle();

                this->checkPanneau(6);
                usleep(150'000);
                this->uncheckPanneau(6);
                break;
            case TURN_SOLAR_PANNEL_2:
                this->go(2525, 1790);
                awaitRobotIdle();

                this->rotate(PI);
                awaitRobotIdle();

                this->checkPanneau(6);
                usleep(150'000);
                this->uncheckPanneau(6);
                break;
            case TURN_SOLAR_PANNEL_3:
                this->go(2300, 1790);
                awaitRobotIdle();

                this->rotate(PI);
                awaitRobotIdle();

                this->checkPanneau(6);
                usleep(150'000);
                this->uncheckPanneau(6);
                break;
            default:
                break;
        }
    }

    this->setSpeed(previousSpeed);
}

void TCPServer::checkpoint(StratPattern sp) {
    this->setSpeed(200);
    if (team == BLUE) {
        switch (sp) {
            case CHECKPOINT_BOTTOM_TO_TOP:
                this->go(500, 1000);
                awaitRobotIdle();
                this->go(500, 500);
                this->awaitRobotIdle();
                break;
            case CHECKPOINT_TOP_TO_BOTTOM:
                this->go(500, 1500);
                this->awaitRobotIdle();
                break;
            case CHECKPOINT_TRANSITION_SOLAR_PANEL_FLOWER:
                this->go(500, 1700);
                awaitRobotIdle();
                break;
            default:
                break;
        }
    } else if (team == YELLOW) {
        switch (sp) {
            case CHECKPOINT_BOTTOM_TO_TOP:
                this->go(2500, 1000);
                awaitRobotIdle();
                this->go(2500, 500);
                this->awaitRobotIdle();
                break;
            case CHECKPOINT_TOP_TO_BOTTOM:
                this->go(2500, 1500);
                this->awaitRobotIdle();
                break;
            case CHECKPOINT_TRANSITION_SOLAR_PANEL_FLOWER:
                this->go(2500, 1700);
                awaitRobotIdle();
                break;
            default:
                break;
        }
    }
}


void TCPServer::getLidarPos() {

    this->askLidarPosition();

    awaitForLidar = true;
    // ReSharper disable once CppDFAConstantConditions
    // ReSharper disable once CppDFAEndlessLoop
    while (awaitForLidar) {
        usleep(200'000);
    }

    // ReSharper disable once CppDFAUnreachableCode
    std::cout << lidarCalculatePos.pos.x << " " << lidarCalculatePos.pos.y << " " << lidarCalculatePos.theta << std::endl;

}


template<class X, class Y>
void TCPServer::go(X x, Y y) {
    this->broadcastMessage("strat;arduino;go;" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "\n");
}

template<class X>
void TCPServer::go(std::array<X, 2> data) {
    this->broadcastMessage("strat;arduino;go;" + std::to_string(static_cast<int>(data[0])) + "," + std::to_string(static_cast<int>(data[1])) + "\n");
}

template<class X>
void TCPServer::rotate(X angle) {
    this->broadcastMessage("strat;arduino;angle;" + std::to_string(static_cast<int>(angle * 100)) + "\n");
}

void TCPServer::setSpeed(const int speed) {
    this->broadcastMessage("strat;arduino;speed;" + std::to_string(speed) + "\n");
    this->speed = speed;
}

template<class X, class Y>
void TCPServer::transit(X x, Y y, const int endSpeed) {
    this->broadcastMessage("strat;arduino;transit;" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "," + std::to_string(endSpeed) + "\n");
}

template<class X>
void TCPServer::transit(std::array<X, 2> data, const int endSpeed) {
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
        this->sendToClient("strat;all;set pos;" + std::to_string(static_cast<int>(pos.pos.x)) + "," + std::to_string(static_cast<int>(pos.pos.y)) + "," + std::to_string(static_cast<int>(pos.theta * 100)) + "\n", clientSocket);
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
