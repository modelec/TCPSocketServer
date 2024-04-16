#include "TCPServer.h"

#include <cmath>
#include <fstream>

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
        // TODO change that to handle spawn point
        /*int spawnPointNb = std::stoi(tokens[3]);
        float spawnPoint[2];
        float finishPoint[2];

        switch (spawnPointNb) {
            case 3:
                this->team = BLUE;
                spawnPoint[0] = 500;
                spawnPoint[1] = 500;
                finishPoint[0] = 500;
                finishPoint[1] = 500;

                break;
            case 6:
                this->team = YELLOW;
                spawnPoint[0] = 500;
                spawnPoint[1] = 500;
                finishPoint[0] = 500;
                finishPoint[1] = 500;
                break;

            default:
                return;
        }

        std::ofstream file("end_point.txt");
        file << finishPoint[0] << " " << finishPoint[1];
        file.close();*/

        // this->robotPose = {500, 500, -1.57079};
        this->robotPose = {1200, 1700, 1.57079};
        this->initRobotPose = {1200, 1700, 1.57079};
        this->endRobotPose = {1200, 1700, 1.57079};
        std::string toSend = "strat;all;set pos;" + std::to_string(this->robotPose.pos.x) + "," + std::to_string(this->robotPose.pos.y) + "," + std::to_string(this->robotPose.theta * 100) + "\n";

        for (int j = 0; j < 3; j++) {
            this->broadcastMessage(toSend);
            usleep(200'000);
        }
    }
    else if (tokens[1] == "strat" && tokens[2] == "start")
    {
        this->broadcastMessage(message.c_str(), clientSocket);
        std::thread([this]() { this->startGame(); }).detach();
    }
    else if (tokens[0] == "aruco" && tokens[2] == "get aruco") {
        std::string arucoResponse = tokens[3];
        if (arucoResponse != "404") {
            this->arucoTags.clear();
            std::vector<std::string> aruco = TCPUtils::split(arucoResponse, ",");
            for (int i = 0; i < aruco.size() - 1; i += 7) {
                ArucoTag tag;
                tag.setId(std::stoi(aruco[i]));
                tag.setName(aruco[i + 1]);

                float pos[2] = {std::stof(aruco[i + 2]), std::stof(aruco[i + 3])};
                float rot[3] = {std::stof(aruco[i + 4]), std::stof(aruco[i + 5]), std::stof(aruco[i + 6])};

                tag.setPos(pos[0], pos[1]);
                tag.setRot(rot[0], rot[1], rot[2]);

                this->arucoTags.push_back(tag);
            }
        }
    }
    else if (tokens[0] == "arduino" && tokens[2] == "set state") {
        if (TCPUtils::startWith(tokens[3], "0")) {
            this->isRobotMoving++;
        }
    }
    std::cout << "Received: " << message << std::endl;
}


void TCPServer::broadcastMessage(const char* message, int senderSocket)
{
    std::string temp = message;
    this->broadcastMessage(temp, senderSocket);
}

void TCPServer::broadcastMessage(std::string &message, int senderSocket) {
    if (!TCPUtils::endsWith(message, "\n")) {
        message += "\n";
    }

    for (int clientSocket : clientSockets) {
        if (clientSocket != senderSocket) { // Exclude the sender's socket
            send(clientSocket, message.c_str(), message.size(), 0);
        }
    }
}

void TCPServer::sendToClient(std::string &message, int clientSocket) {
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

void TCPServer::sendToClient(std::string &message, const std::string &clientName) {
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
        this->broadcastMessage("strat;all;ready;1");
        std::thread([this]() { askArduinoPos(); }).detach();
    }
}

void TCPServer::startGame() {
    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
    this->broadcastMessage("strat;servo_moteur;fermer pince;1\n");
    this->broadcastMessage("strat;servo_moteur;fermer pince;2\n");
    this->broadcastMessage("strat;servo_moteur;ouvrir pince;0\n");
    // TODO set to 200 when the robot is ready
    this->broadcastMessage("strat;arduino;speed;150\n");

    this->broadcastMessage("strat;aruco;get aruco;1\n");

    int timeout = 0;
    ArucoTag tag;
    bool found = false;
    while (!found) {
        for (const auto & arucoTag : this->arucoTags) {
            if (TCPUtils::endsWith(arucoTag.name(), "flower")) {
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
    isRobotMoving = 0;
    while (this->isRobotMoving < 4) {
        usleep(200'000);
        this->broadcastMessage("strat;arduino;get state;1\n");
    }
    usleep(500'000);

    // ReSharper disable once CppDFAUnreachableCode
    usleep(500'000);

    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");

    usleep(2'000'000);
    arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");

    found = false;
    timeout = 0;
    while (!found) {
        for (const auto & arucoTag : this->arucoTags) {
            if (TCPUtils::endsWith(arucoTag.name(), "flower")) {
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
    isRobotMoving = 0;
    while (this->isRobotMoving < 4) {
        usleep(200'000);
        this->broadcastMessage("strat;arduino;get state;1\n");
    }
    usleep(500'000);

    // ReSharper disable once CppDFAUnreachableCode
    usleep(500'000);

    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");

    usleep(2'000'000);
    arucoTags.clear();
    this->broadcastMessage("strat;aruco;get aruco;1\n");

    found = false;
    timeout = 0;
    while (!found) {
        for (const auto & arucoTag : this->arucoTags) {
            if (TCPUtils::endsWith(arucoTag.name(), "flower")) {
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

    std::string toSend = "strat;arduino;go;762,200\n";
    this->broadcastMessage(toSend);
    usleep(5'000'000);
    /*isRobotMoving = 0;
    while (this->isRobotMoving < 4) {
        usleep(200'000);
        this->broadcastMessage("strat;arduino;get state;1\n");
    }
    usleep(500'000);*/

    this->broadcastMessage("strat;arduino;angle;0\n");
    usleep(5'000'000);
    /*isRobotMoving = 0;
    while (this->isRobotMoving < 4) {
        usleep(200'000);
        this->broadcastMessage("strat;arduino;get state;1\n");
    }
    usleep(500'000);*/

    this->broadcastMessage("strat;arduino;speed;150\n");
    this->broadcastMessage("strat;arduino;go;762,0\n");
    usleep(5'000'000);

    this->broadcastMessage("strat;servo_moteur;ouvrir pince;0\n");
    pinceState[0] = NONE;
    this->broadcastMessage("strat;servo_moteur;ouvrir pince;2\n");
    pinceState[2] = NONE;
    usleep(1'000'000);

    this->broadcastMessage("strat;servo_moteur;fermer pince;0\n");
    this->broadcastMessage("strat;servo_moteur;fermer pince;2\n");
    this->broadcastMessage("strat;servo_moteur;ouvrir pince;1\n");
    pinceState[1] = NONE;
    usleep(1'000'000);

    toSend = "strat;arduino;go;" + std::to_string(static_cast<int>(this->endRobotPose.pos.x)) + "," + std::to_string(static_cast<int>(this->endRobotPose.pos.y)) + "\n";
    this->broadcastMessage(toSend);
    isRobotMoving = 0;
    while (this->isRobotMoving < 3) {
        usleep(200'000);
        this->broadcastMessage("strat;arduino;get state;1\n");
    }
    usleep(500'000);

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
            decalage = 80;
            rotate = -0.1;
            break;
        case 1:
            decalage = 0;
            rotate = 0;
            break;
        case 2:
            decalage = -80;
            rotate = 0.1;
            break;
        default:
            decalage = 0;
            rotate = 0;
            break;
    }

    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
    std::string toSend = "strat;servo_moteur;ouvrir pince;" + std::to_string(pince) + "\n";

    this->broadcastMessage(toSend);

    double xPrime = arucoTag.pos().first[0] + 20;
    double yPrime = arucoTag.pos().first[1] + decalage;

    xPrime *= 1.1;
    yPrime *= 1.1;

    double thetaPrime = std::atan2(yPrime, xPrime);

    toSend = "strat;arduino;angle;" + std::to_string(static_cast<int>((this->robotPose.theta + rotate - thetaPrime) * 100)) + "\n";
    this->broadcastMessage(toSend);
    isRobotMoving = 0;
    while (this->isRobotMoving < 3) {
        usleep(200'000);
        this->broadcastMessage("strat;arduino;get state;1\n");
    }
    usleep(500'000);

    double x30Percent = xPrime * 0.3;
    double y30Percent = yPrime * 0.3;

    xPrime -= x30Percent;
    yPrime -= y30Percent;

    double pos30PercentX = (xPrime * std::cos(theta) + yPrime * std::sin(theta)) + robotPosX;
    double pos30PercentY = (-xPrime * std::sin(theta) + yPrime * std::cos(theta)) + robotPosY;

    toSend = "strat;arduino;go;" + std::to_string(static_cast<int>(pos30PercentX)) + "," + std::to_string(static_cast<int>(pos30PercentY)) + "\n";
    this->broadcastMessage(toSend);
    isRobotMoving = 0;
    while (this->isRobotMoving < 3) {
        usleep(200'000);
        this->broadcastMessage("strat;arduino;get state;1\n");
    }
    usleep(500'000);

    // ReSharper disable once CppDFAUnreachableCode
    // TODO set to 150 when the robot is ready
    this->broadcastMessage("strat;arduino;speed;130\n");
    usleep(1'000'000);

    xPrime += x30Percent;
    yPrime += y30Percent;

    double robotPosForPotX = (xPrime * std::cos(theta) + yPrime * std::sin(theta)) + robotPosX;
    double robotPosForPotY = (-xPrime * std::sin(theta) + yPrime * std::cos(theta)) + robotPosY;

    toSend = "strat;arduino;go;" + std::to_string(static_cast<int>(robotPosForPotX)) + "," + std::to_string(static_cast<int>(robotPosForPotY)) + "\n";
    this->broadcastMessage(toSend);
    isRobotMoving = 0;
    while (this->isRobotMoving < 3) {
        usleep(200'000);
        this->broadcastMessage("strat;arduino;get state;1\n");
    }
    usleep(3'000'000);

    toSend = "strat;servo_moteur;fermer pince;" + std::to_string(pince) + "\n";
    this->broadcastMessage(toSend);
    usleep(500'000);
    this->broadcastMessage("strat;servo_moteur;lever bras;1\n");
    // TODO set to 200 when the robot is ready
    this->broadcastMessage("strat;arduino;speed;150\n");
    pinceState[pince] = arucoTag.name() == "Purple_flower" ? PURPLE_FLOWER : WHITE_FLOWER;
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
        usleep(500'000);
    }
}

void TCPServer::turnSolarPanel(const ArucoTag &arucoTag) {

    if (arucoTag.name() != "Solar_Panel") {
        return;
    }

    double robotPosX = this->robotPose.pos.x;
    double robotPosY = this->robotPose.pos.y;
    double theta = this->robotPose.theta;

    double xPrime = arucoTag.pos().first[0];
    double yPrime = arucoTag.pos().first[1];

    double solarPanelX = (xPrime * std::cos(theta) + yPrime * std::sin(theta)) + robotPosX;
    double solarPanelY = (-xPrime * std::sin(theta) + yPrime * std::cos(theta)) + robotPosY;

    double yaw = arucoTag.pos().second[2];

    solarPanelX -= 100;

    std::string toSend = "strat;arduino;go;" + std::to_string(static_cast<int>(solarPanelX)) + "," + std::to_string(static_cast<int>(solarPanelY)) + "\n";

    this->broadcastMessage(toSend);
}
