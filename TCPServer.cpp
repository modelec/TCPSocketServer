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
            //std::cout << "Received: " << buffer << std::endl;

            if (buffer == "quit") {
                std::cerr << "Client requested to quit. Closing connection." << std::endl;
                break;
            }
            processMessage(buffer);

            buffer.clear();
        } else if (valread == 0) {
            std::cout << "Client disconnected." << std::endl;
            break; // Client disconnected
        } else {
            std::cerr << "Failed to receive data." << std::endl;
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

TCPServer::TCPServer(int port)
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
    ClientTCP tirette;
    tirette.name = "tirette";

    ClientTCP aruco;
    aruco.name = "aruco";

    ClientTCP ihm;
    ihm.name = "ihm";

    ClientTCP lidar;
    lidar.name = "lidar";

    ClientTCP arduino;
    arduino.name = "arduino";

    ClientTCP servo_moteur;
    servo_moteur.name = "servo_moteur";

    clients.push_back(tirette);
    clients.push_back(aruco);
    clients.push_back(ihm);
    clients.push_back(lidar);
    clients.push_back(arduino);
    clients.push_back(servo_moteur);
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
    // std::cout << message << std::endl;

    std::vector<std::string> tokens = TCPUtils::split(message, ";");

    if (tokens.size() != 4)
    {
        std::cerr << "Invalid message format : " << message << std::endl;
        return;
    }
    if (tokens[1] != "strat")
    {
        this->broadcastMessage(message.c_str(), clientSocket);
    }

    // EMERGENCY
    if (tokens[2] == "stop proximity") {
        // TODO handle emergency
        // this->broadcastMessage("strat;arduino;stop;1");
    }

    else if (tokens[0] == "tirette" && tokens[2] == "set state")
    {
        this->broadcastMessage(message.c_str(), clientSocket);
    }
    else if (tokens[2] == "ready")
    {
        for (ClientTCP& client : clients)
        {
            if (client.name == tokens[0])
            {
                client.isReady = true;
                client.socket = clientSocket;
                break;
            }
        }
        checkIfAllClientsReady();
    }
    else if (tokens[2] == "set pos") {
        std::vector<std::string> pos = TCPUtils::split(tokens[3], ",");
        this->robotPose = {std::stof(pos[0]), std::stof(pos[1]), std::stof(pos[2]) / 100};
    }
    else if (tokens[2] == "get pos") {
        std::string toSend = "strat;all;set pos;" + std::to_string(this->robotPose.pos.x) + "," + std::to_string(this->robotPose.pos.y) + "," + std::to_string(this->robotPose.theta * 100) + "\n";
        this->sendToClient(toSend, clientSocket);
    }
    else if (tokens[2] == "spawn") {
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
                finishPoint[0] = 400;
                finishPoint[1] = 1790;
                finishPoint[2] = 0;

                /*finishPoint[0] = 400;
                finishPoint[1] = 400;
                finishPoint[2] = 3.1415;*/
                break;
            case 6:
                this->team = YELLOW;
                spawnPoint[0] = 1750;
                spawnPoint[1] = 1790;
                spawnPoint[2] = 3.1415;
                finishPoint[0] = 2600;
                finishPoint[1] = 400;
                finishPoint[2] = 0;
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
        std::string toSend = "strat;all;set pos;" + std::to_string(this->robotPose.pos.x) + "," + std::to_string(this->robotPose.pos.y) + "," + std::to_string(this->robotPose.theta * 100) + "\n";

        for (int j = 0; j < 3; j++) {
            this->broadcastMessage(toSend);
            usleep(200'000);
        }
    }
    else if (tokens[1] == "strat" && tokens[2] == "start")
    {
        this->broadcastMessage(message.c_str(), clientSocket);

        switch (this->team) {
            case BLUE:
                std::thread([this]() { this->startGameBlueTeam(); }).detach();
                break;
            case YELLOW:
                std::thread([this]() { this->startGameYellowTeam(); }).detach();
                break;
            case TEST:
                std::thread([this]() { this->startGameTest(); }).detach();
                break;
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

                handleArucoTag(tag);
            }
        }
    }
    else if (tokens[0] == "arduino") {
        if (tokens[2] == "set state") {
            if (TCPUtils::startWith(tokens[3], "0")) {
                this->isRobotIdle++;
            }
        } else if (tokens[2] == "set speed") {
            this->speed = std::stoi(tokens[3]);
        }
    } else if (tokens[2] == "get speed") {
        this->sendToClient("strat;" + tokens[0] + ";set speed;" + std::to_string(this->speed) + "\n", clientSocket);
    }
    std::cout << "Received: " << message << std::endl;
}

void TCPServer::broadcastMessage(const char* message, int senderSocket)
{
    for (int clientSocket : clientSockets) {
        if (clientSocket != senderSocket) { // Exclude the sender's socket
            send(clientSocket, message, strlen(message), 0);
        }
    }
}

void TCPServer::broadcastMessage(const std::string &message, int senderSocket) {
    this->broadcastMessage(message.c_str(), senderSocket);
}

void TCPServer::sendToClient(const std::string &message, int clientSocket) {
    this->sendToClient(message.c_str(), clientSocket);
}

void TCPServer::sendToClient(const char *message, int clientSocket) {
    for (int socket : clientSockets) {
        if (socket == clientSocket) {
            send(socket, message, strlen(message), 0);
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

void TCPServer::startGameBlueTeam() {
    this->setSpeed(130);
    this->broadcastMessage("strat;servo_moteur;check panneau;7\n");
    usleep(100'000);
    std::string toSend;
    this->go(380, this->robotPose.pos.y);
    awaitRobotIdle();

    this->broadcastMessage("strat;servo_moteur;uncheck panneau;7\n");
    this->go(474, this->robotPose.pos.y);
    awaitRobotIdle();

    this->rotate(0);
    awaitRobotIdle();

    this->broadcastMessage("strat;servo_moteur;check panneau;7\n");
    usleep(100'000);

    this->go(620, this->robotPose.pos.y);
    awaitRobotIdle();

    this->broadcastMessage("strat;servo_moteur;uncheck panneau;7\n");
    this->go(749, this->robotPose.pos.y);
    awaitRobotIdle();

    this->rotate(0);
    awaitRobotIdle();

    this->broadcastMessage("strat;servo_moteur;check panneau;7\n");
    usleep(100'000);

    this->go(1000, 1700);
    awaitRobotIdle();

    this->broadcastMessage("strat;servo_moteur;uncheck panneau;7\n");

    this->rotate(PI / 2);
    awaitRobotIdle();
    this->setSpeed(200);

    this->go(1000, 1800);
    awaitRobotIdle();

    this->rotate(PI / 2);

    usleep(1'000'000);

    this->arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");
    for (int i = 0; i < 5; i++) {
        usleep(500'000);
        this->broadcastMessage("strat;aruco;get aruco;1\n");
    }

    std::optional<ArucoTag> tag = getBiggestArucoTag(300, 700, -200, 200);

    if (tag.has_value()) {
        if (pinceState[1] == NONE) {
            goToAruco(tag.value(), 1);
        } else if (pinceState[2] == NONE) {
            goToAruco(tag.value(), 2);
        } else if (pinceState[0] == NONE) {
            goToAruco(tag.value(), 0);
        }
    }

    this->go(1000, 1700);
    awaitRobotIdle();

    this->rotate(PI / 2);
    awaitRobotIdle();

    usleep(1'000'000);

    this->arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");
    for (int i = 0; i < 5; i++) {
        usleep(500'000);
        this->broadcastMessage("strat;aruco;get aruco;1\n");
    }

    tag = getBiggestArucoTag(300, 700, -200, 200);

    if (tag.has_value()) {
        if (pinceState[1] == NONE) {
            goToAruco(tag.value(), 1);
        } else if (pinceState[2] == NONE) {
            goToAruco(tag.value(), 2);
        } else if (pinceState[0] == NONE) {
            goToAruco(tag.value(), 0);
        }
    }

    this->go(1000, 1700);
    awaitRobotIdle();

    this->rotate(PI / 2);
    awaitRobotIdle();

    usleep(1'000'000);

    this->arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");
    for (int i = 0; i < 5; i++) {
        usleep(500'000);
        this->broadcastMessage("strat;aruco;get aruco;1\n");
    }

    tag = getBiggestArucoTag(300, 700, -200, 200);

    if (tag.has_value()) {
        if (pinceState[1] == NONE) {
            goToAruco(tag.value(), 1);
        } else if (pinceState[2] == NONE) {
            goToAruco(tag.value(), 2);
        } else if (pinceState[0] == NONE) {
            goToAruco(tag.value(), 0);
        }
    }

    this->go(1000, 1700);
    awaitRobotIdle();

    this->rotate(PI);
    awaitRobotIdle();

    usleep(1'000'000);

    this->go(this->robotPose.pos.x - 350, this->robotPose.pos.y);
    awaitRobotIdle();

    this->go(500, 500);
    awaitRobotIdle();

    std::vector<int> pinceHavePurpleFlower;
    for (int i = 0; i < 3; i++) {
        if (pinceState[i] == PURPLE_FLOWER) {
            pinceHavePurpleFlower.push_back(i);
        }
    }

    if (!pinceHavePurpleFlower.empty()) {
        this->go(300, 300);
        awaitRobotIdle();
        this->rotate(PI / 2);
        awaitRobotIdle();

        for (auto & toDrop : pinceHavePurpleFlower) {
            toSend = "strat;servo_moteur;ouvrir pince;" + std::to_string(toDrop) + "\n";
            this->broadcastMessage(toSend);

            this->go(this->robotPose.pos.x, this->robotPose.pos.y + 150);
            awaitRobotIdle();

            pinceState[toDrop] = NONE;
            toSend = "strat;servo_moteur;fermer pince;" + std::to_string(toDrop) + "\n";
            this->broadcastMessage(toSend);
            usleep(100'000);
        }

        this->go(this->robotPose.pos.x + 150, this->robotPose.pos.y);
        awaitRobotIdle();
    }


    if (pinceHavePurpleFlower.size() < 3) {
        this->go(762, 300);
        awaitRobotIdle();

        this->setSpeed(130);
        this->rotate(PI / 2);
        awaitRobotIdle();

        this->broadcastMessage("strat;servo_moteur;lever bras;1\n");

        this->go(762, 0);
        usleep(3'000'000);

        for (int i = 0; i < 3; i++) {
            if (pinceState[i] == WHITE_FLOWER) {
                toSend = "strat;servo_moteur;ouvrir pince;" + std::to_string(i) + "\n";
                this->broadcastMessage(toSend);
                usleep(500'000);
                pinceState[i] = NONE;
                toSend = "strat;servo_moteur;fermer pince;" + std::to_string(i) + "\n";
                this->broadcastMessage(toSend);
                usleep(100'000);
            }
        }

        this->setSpeed(200);
    }

    /*
     *  TODO
     *  here add the whole strategy, for the moment the robot only took 3 plants and drop them
     *  Handle the second plant spot
     */

    this->go(1000, 250);
    awaitRobotIdle();

    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");

    this->rotate(-PI/4);
    awaitRobotIdle();

    this->arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");
    for (int i = 0; i < 5; i++) {
        usleep(500'000);
        this->broadcastMessage("strat;aruco;get aruco;1\n");
    }

    tag = getBiggestArucoTag(300, 700, -200, 200);

    if (tag.has_value()) {
        if (pinceState[1] == NONE) {
            goToAruco(tag.value(), 1);
        } else if (pinceState[2] == NONE) {
            goToAruco(tag.value(), 2);
        } else if (pinceState[0] == NONE) {
            goToAruco(tag.value(), 0);
        }
    }

    this->go(1000, 250);
    awaitRobotIdle();

    this->rotate(-PI / 2);
    awaitRobotIdle();

    this->arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");
    for (int i = 0; i < 5; i++) {
        usleep(500'000);
        this->broadcastMessage("strat;aruco;get aruco;1\n");
    }

    tag = getBiggestArucoTag(300, 700, -200, 200);

    if (tag.has_value()) {
        if (pinceState[1] == NONE) {
            goToAruco(tag.value(), 1);
        } else if (pinceState[2] == NONE) {
            goToAruco(tag.value(), 2);
        } else if (pinceState[0] == NONE) {
            goToAruco(tag.value(), 0);
        }
    }


    this->go(1000, 250);
    awaitRobotIdle();

    this->rotate(PI / 2);
    awaitRobotIdle();

    this->arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");
    for (int i = 0; i < 5; i++) {
        usleep(500'000);
        this->broadcastMessage("strat;aruco;get aruco;1\n");
    }

    tag = getBiggestArucoTag(300, 700, -200, 200);

    if (tag.has_value()) {
        if (pinceState[1] == NONE) {
            goToAruco(tag.value(), 1);
        } else if (pinceState[2] == NONE) {
            goToAruco(tag.value(), 2);
        } else if (pinceState[0] == NONE) {
            goToAruco(tag.value(), 0);
        }
    }

    this->go(1000, 250);
    awaitRobotIdle();

    this->rotate(PI);
    awaitRobotIdle();

    pinceHavePurpleFlower.clear();
    for (int i = 0; i < 3; i++) {
        if (pinceState[i] == PURPLE_FLOWER) {
            pinceHavePurpleFlower.push_back(i);
        }
    }

    if (!pinceHavePurpleFlower.empty()) {
        this->broadcastMessage("strat;servo_moteur;lever bras;1\n");

        this->go(225, 225);
        awaitRobotIdle();
        this->rotate(PI);
        awaitRobotIdle();

        for (auto & toDrop : pinceHavePurpleFlower) {
            toSend = "strat;servo_moteur;ouvrir pince;" + std::to_string(toDrop) + "\n";
            this->broadcastMessage(toSend);
            usleep(500'000);
            pinceState[toDrop] = NONE;
            toSend = "strat;servo_moteur;fermer pince;" + std::to_string(toDrop) + "\n";
            this->broadcastMessage(toSend);
            usleep(100'000);
        }

        this->go(this->robotPose.pos.x + 150, this->robotPose.pos.y);
        awaitRobotIdle();
    }


    if (pinceHavePurpleFlower.size() < 3) {
        this->go(762, 300);
        awaitRobotIdle();

        this->rotate(PI / 2);
        awaitRobotIdle();

        this->setSpeed(150);
        this->go(762, 0);
        usleep(1'000'000);


        for (int i = 0; i < 3; i++) {
            if (pinceState[i] == WHITE_FLOWER) {
                toSend = "strat;servo_moteur;ouvrir pince;" + std::to_string(i) + "\n";
                this->broadcastMessage(toSend);
                usleep(200'000);
                pinceState[i] = NONE;
                toSend = "strat;servo_moteur;fermer pince;" + std::to_string(i) + "\n";
                this->broadcastMessage(toSend);
                usleep(100'000);
            }

            this->setSpeed(200);

            this->go(this->robotPose.pos.x, this->robotPose.pos.y + 350);
            awaitRobotIdle();
        }
    }


    this->go(this->endRobotPose.pos.x, this->endRobotPose.pos.y);
    awaitRobotIdle();

    this->rotate(this->endRobotPose.theta);
    awaitRobotIdle();

    this->broadcastMessage("strat;servo_moteur;clear;1");
}


void TCPServer::startGameYellowTeam() {
    this->broadcastMessage("strat;arduino;speed;200\n");
    this->broadcastMessage("strat;servo_moteur;check panneau;7\n");
    usleep(100'000);
    std::string toSend = "strat;arduino;go;";
    this->broadcastMessage(toSend);


    toSend = "strat;arduino;go;" + std::to_string(static_cast<int>(this->endRobotPose.pos.x)) + "," + std::to_string(static_cast<int>(this->endRobotPose.pos.y)) + "\n";
    this->broadcastMessage(toSend);
    awaitRobotIdle();
    toSend = "strat;arduino;angle;" + std::to_string(static_cast<int>(this->endRobotPose.theta * 100));
    this->broadcastMessage("strat;arduino;speed;150\n");
    this->broadcastMessage(toSend);
    awaitRobotIdle();
    this->broadcastMessage("strat;arduino;speed;200\n");

    this->broadcastMessage("strat;servo_moteur;clear;1");
}

void TCPServer::startGameTest() {
    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
    this->broadcastMessage("strat;servo_moteur;fermer pince;1\n");
    this->broadcastMessage("strat;servo_moteur;fermer pince;2\n");
    this->broadcastMessage("strat;servo_moteur;ouvrir pince;0\n");
    // TODO set to 200 when the robot is ready
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
    double robotPosX = this->robotPose.pos.x;
    double robotPosY = this->robotPose.pos.y;
    double theta = this->robotPose.theta;
    double decalage;
    double rotate;
    if (pince < 0 || pince > 2) {
        return;
    }

    switch (pince) {
        case 0:
            decalage = 60;
            rotate = -0.07;
            break;
        case 1:
            decalage = 0;
            rotate = 0;
            break;
        case 2:
            decalage = -60;
            rotate = 0.07;
            break;
        default:
            decalage = 0;
            rotate = 0;
            break;
    }

    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
    std::string toSend = "strat;servo_moteur;ouvrir pince;" + std::to_string(pince) + "\n";

    this->broadcastMessage(toSend);

    double xPrime = arucoTag.pos()[0] - 50;
    double yPrime = arucoTag.pos()[1] + decalage;

    // TODO : changement de base pour prendre en compte le centre de la plante avec la matrice de rotation,

    double thetaPrime = std::atan2(yPrime, xPrime);

    this->rotate(this->robotPose.theta + rotate - thetaPrime);
    awaitRobotIdle();

    double robotPosForPotX = (xPrime * std::cos(theta) + yPrime * std::sin(theta)) + robotPosX;
    double robotPosForPotY = (-xPrime * std::sin(theta) + yPrime * std::cos(theta)) + robotPosY;

    int previousSpeed = this->speed;

    this->transit(robotPosX, robotPosY, 130);
    awaitRobotIdle();

    toSend = "strat;servo_moteur;fermer pince;" + std::to_string(pince) + "\n";
    this->broadcastMessage(toSend);
    usleep(500'000);
    this->setSpeed(previousSpeed);
    pinceState[pince] = TCPUtils::startWith(arucoTag.name(), "Purple_flower") ? PURPLE_FLOWER : WHITE_FLOWER;
}

void TCPServer::askArduinoPos() {
    ClientTCP arduino;
    for (const auto & client : clients) {
        if (client.name == "arduino") {
            arduino = client;
            break;
        }
    }

    if (arduino.socket == -1) {
        return;
    }

    while (!this->_shouldStop) {
        this->sendToClient("strat;arduino;get pos;1\n", arduino.socket);
        usleep(200'000);
    }
}

void TCPServer::awaitRobotIdle() {
    isRobotIdle = 0;
    int timeout = 0;
    // ReSharper disable once CppDFAConstantConditions
    // ReSharper disable once CppDFAEndlessLoop
    usleep(200'000);
    while (isRobotIdle < 3) {
        usleep(200'000);
        this->broadcastMessage("strat;arduino;get state;1\n");
        timeout++;
        if (timeout > 30) {
            this->broadcastMessage("strat;arduino;clear;1");
        }
    }
}

void TCPServer::handleArucoTag(ArucoTag &tag) {
    if (!TCPUtils::endWith(tag.name(), "flower")) {
        return;
    }

    for (auto& t : arucoTags) {
        if (tag.id() == t.id()) {
            float tPosX = t.pos()[0];
            float tPosY = t.pos()[1];
            float tagPosX = tag.pos()[0];
            float tagPosY = tag.pos()[1];
            if (tagPosX > tPosX - 1 && tagPosX < tPosX + 1 && tagPosY > tPosY - 1 && tagPosY < tPosY + 1) {
                t.find();
                break;
            }
        }
    }

    this->arucoTags.push_back(tag);
}

std::optional<ArucoTag> TCPServer::getBiggestArucoTag(float borneMinX, float borneMaxX, float borneMinY,
                                                      float borneMaxY) {
    // use optional
    bool found = false;
    ArucoTag biggestTag = ArucoTag();
    for (const auto & tag : arucoTags) {
        if (tag.getNbFind() > biggestTag.getNbFind() && tag.pos()[0] > borneMinX && tag.pos()[0] < borneMaxX && tag.pos()[1] > borneMinY && tag.pos()[1] < borneMaxY) {
            biggestTag = tag;
            found = true;
        }
    }

    return found ? std::optional<ArucoTag>(biggestTag) : std::nullopt;
}


template<class X, class Y>
void TCPServer::go(X x, Y y) {
    this->broadcastMessage("strat;arduino;go;" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "\n");
}

template<class X>
void TCPServer::rotate(X angle) {
    this->broadcastMessage("strat;arduino;angle" + std::to_string(static_cast<int>(angle * 100)) + "\n");
}

void TCPServer::setSpeed(int speed) {
    this->broadcastMessage("strat;arduino;speed;" + std::to_string(speed) + "\n");
    this->speed = speed;
}

template<class X, class Y>
void TCPServer::transit(X x, Y y, int endSpeed) {
    this->broadcastMessage("strat;arduino;transit" + std::to_string(static_cast<int>(x)) + "," + std::to_string(static_cast<int>(y)) + "," + std::to_string(endSpeed) + "\n");
}
