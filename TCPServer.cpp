#include "TCPServer.h"

#include <cmath>

ClientHandler::ClientHandler(int clientSocket, TCPServer* server) : clientSocket(clientSocket), server(server) {};

void ClientHandler::handle() {
    std::string buffer;
    buffer.reserve(4096); // Pre-allocate memory to avoid frequent allocations

    while (true) {
        char tempBuffer[4096] = {0};
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
    std::cout << message << " | " << clientSocket << std::endl;

    std::vector<std::string> tokens = split(message, ";");

    if (tokens.size() != 4)
    {
        std::cerr << "Invalid message format : " << message << std::endl;
        return;
    }
    if (tokens[1] != "strat")
    {
        this->broadcastMessage(message.c_str(), clientSocket);
    }
    if (tokens[0] == "tirette" && tokens[2] == "set state")
    {
        this->broadcastMessage(message.c_str(), clientSocket);
    }
    if (tokens[2] == "ready")
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
    if (tokens[2] == "set pos") {
        std::vector<std::string> pos = split(tokens[3], ",");
        this->robotPose = {std::stof(pos[0]), std::stof(pos[1]), std::stof(pos[2]) / 100};
    }
    if (tokens[2] == "get pos") {
        std::string toSend = "strat;all;set pos;" + std::to_string(this->robotPose.pos.x) + "," + std::to_string(this->robotPose.pos.y) + "," + std::to_string(this->robotPose.theta * 100) + "\n";
        this->sendToClient(toSend, clientSocket);
    }
    if (tokens[2] == "spawn") {
        // TODO change that to handle spawn point
        this->robotPose = {500, 500, -1.57079};
        std::string toSend = "strat;all;set pos;" + std::to_string(this->robotPose.pos.x) + "," + std::to_string(this->robotPose.pos.y) + "," + std::to_string(this->robotPose.theta * 100) + "\n";
        this->broadcastMessage(toSend, clientSocket);
    }
    if (tokens[2] == "start" && tokens[1] == "strat")
    {
        this->broadcastMessage(message.c_str(), clientSocket);
        this->broadcastMessage("strat;arduino;speed;200\n");
        this->actionNb = 0;
        std::thread([this]() { this->startGame(); }).detach();
    }
    if (tokens[0] == "aruco" && tokens[2] == "get aruco") {
        std::string arucoResponse = tokens[3];
        if (arucoResponse != "404") {
            this->arucoTags.clear();
            std::vector<std::string> aruco = split(arucoResponse, ",");
            for (int i = 0; i < aruco.size() - 1; i += 7) {
                ArucoTagPos tag;
                tag.tag.id = std::stoi(aruco[i]);
                tag.tag.name = aruco[i + 1];
                tag.pos.first[0] = std::stof(aruco[i + 2]);
                tag.pos.first[1] = std::stof(aruco[i + 3]);
                tag.pos.second[0] = std::stof(aruco[i + 4]);
                tag.pos.second[1] = std::stof(aruco[i + 5]);
                tag.pos.second[2] = std::stof(aruco[i + 6]);

                this->arucoTags.push_back(tag);
            }
        }
        this->waitForAruco = false;
    }
    // std::cout << "Received: " << message << std::endl;
}


void TCPServer::broadcastMessage(const char* message, int senderSocket)
{
    std::string temp = message;
    this->broadcastMessage(temp, senderSocket);
}

void TCPServer::broadcastMessage(std::string &message, int senderSocket) {
    if (!endsWith(message, "\n")) {
        message += "\n";
    }

    for (int clientSocket : clientSockets) {
        if (clientSocket != senderSocket) { // Exclude the sender's socket
            send(clientSocket, message.c_str(), message.size(), 0);
        }
    }
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
    // Close the server socket
    close(serverSocket);
}

TCPServer::~TCPServer() {
    this->stop();
    // Join all threads before exiting
    for (auto& thread : clientThreads) {
        thread.join();
    }
}

int TCPServer::nbClients() const {
    return connectedClients;
}

void TCPServer::start()
{
    std::thread([this]() { acceptConnections(); }).detach();
    std::thread([this]() { askArduinoPos(); }).detach();
}

void TCPServer::checkIfAllClientsReady()
{
    bool allReady = true;
    for (auto&[name, socket, isReady] : clients)
    {
        if (!isReady)
        {
            std::cout << name << " is not ready" << std::endl;
            allReady = false;
        }
    }

    if (allReady)
    {
        this->broadcastMessage("strat;all;ready;1");
    }
}

void TCPServer::startGame() {
    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
    this->broadcastMessage("strat;servo_moteur;fermer pince;1\n");
    this->broadcastMessage("strat;servo_moteur;fermer pince;2\n");
    this->broadcastMessage("strat;servo_moteur;ouvrir pince;0\n");
    this->broadcastMessage("strat;arduino;speed;200\n");

    waitForAruco = true;
    this->broadcastMessage("strat;aruco;get aruco;1\n");
    // std::cout << "Ask for aruco" << std::endl;

    while (this->waitForAruco) {
        usleep(100'000);
    }

    // ReSharper disable once CppDFAUnreachableCode
    if (this->arucoTags.empty()) {
        return;
        // TODO maybe rotate the robot but handle that here
    }

    // std::cout << "Position " << this->robotPose.pos.x << " " << this->robotPose.pos.y << " " << this->robotPose.theta << std::endl;
    // std::cout << "Aruco found" << std::endl;
    goToAruco(this->arucoTags[0], 1);
    // std::cout << "I'm on aruco" << std::endl;
    // std::cout << "Position " << this->robotPose.pos.x << " " << this->robotPose.pos.y << " " << this->robotPose.theta << std::endl;

    // pi/4
    this->broadcastMessage("strat;arduino;angle;0\n");
    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
    // std::cout << "Position " << this->robotPose.pos.x << " " << this->robotPose.pos.y << " " << this->robotPose.theta << std::endl;
    usleep(2'000'000);
    // std::cout << "An other aruco" << std::endl;

    waitForAruco = true;
    this->broadcastMessage("strat;aruco;get aruco;1\n");
    while (this->waitForAruco) {
        usleep(100'000);
    }

    if (this->arucoTags.empty()) {
        return;
    }

    // std::cout << "Position " << this->robotPose.pos.x << " " << this->robotPose.pos.y << " " << this->robotPose.theta << std::endl;
    // std::cout << "Aruco found" << std::endl;
    goToAruco(this->arucoTags[0], 2);
    // std::cout << "I'm on aruco" << std::endl;
    // std::cout << "Position " << this->robotPose.pos.x << " " << this->robotPose.pos.y << " " << this->robotPose.theta << std::endl;

    this->broadcastMessage("strat;arduino;rotate;-78\n");
    usleep(1'000'000);
    this->broadcastMessage("strat;arduino;go;500,500\n");
    usleep(6'000'000);
    // std::cout << "I'm on spawn point" << std::endl;
    // -pi/2
    this->broadcastMessage("strat;arduino;angle;-157");
    this->broadcastMessage("strat;servo_moteur;baisser bras;1");
    this->broadcastMessage("strat;servo_moteur;clear;1");
    // std::cout << "End of game" << std::endl;
}

void TCPServer::goToAruco(const ArucoTagPos &arucoTagPos, const int pince) {
    double robotPosX = this->robotPose.pos.x;
    double robotPosY = this->robotPose.pos.y;
    double theta = this->robotPose.theta;
    double decalage;
    if (pince < 0 || pince > 2) {
        return;
    }

    switch (pince) {
        case 0:
            decalage = -80;
            break;
        case 1:
            decalage = 0;
            break;
        case 2:
            decalage = 80;
            break;
        default:
            decalage = 0;
            break;
    }

    this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
    std::string toSend = "strat;servo_moteur;ouvrir pince;" + std::to_string(pince) + "\n";

    this->broadcastMessage(toSend);

    double xPrime = arucoTagPos.pos.first[0] - 5;
    double yPrime = arucoTagPos.pos.first[1] - decalage;

    double x5Percent = xPrime * 0.05;
    double decalage5Percent = decalage * 0.05;

    xPrime -= x5Percent;
    yPrime -= decalage5Percent;

    // std::cout << "Aruco position1 " << xPrime << " " << yPrime << std::endl;

    double posV200X = (xPrime * std::cos(theta) + yPrime * std::sin(theta)) + robotPosX;
    double posV200Y = (-xPrime * std::sin(theta) + yPrime * std::cos(theta)) + robotPosY;

    toSend = "strat;arduino;go;" + std::to_string(static_cast<int>(posV200X)) + "," + std::to_string(static_cast<int>(posV200Y)) + "\n";
    this->broadcastMessage(toSend);
    usleep(4'000'000);
    this->broadcastMessage("strat;arduino;speed;150\n");
    usleep(500'000);

    xPrime += x5Percent;
    yPrime += decalage5Percent;

    double robotPosForPotX = (xPrime * std::cos(theta) + yPrime * std::sin(theta)) + robotPosX;
    double robotPosForPotY = (-xPrime * std::sin(theta) + yPrime * std::cos(theta)) + robotPosY;

    std::cout << "Aruco position " << robotPosForPotX << " " << robotPosForPotY << std::endl;

    toSend = "strat;arduino;go;" + std::to_string(static_cast<int>(robotPosForPotX)) + "," + std::to_string(static_cast<int>(robotPosForPotY)) + "\n";
    this->broadcastMessage(toSend);
    usleep(3'000'000);
    std::cout << "end sleep" << std::endl;
    toSend = "strat;servo_moteur;fermer pince;" + std::to_string(pince) + "\n";
    this->broadcastMessage(toSend);
    this->broadcastMessage("strat;servo_moteur;lever bras;1\n");
    this->broadcastMessage("strat;arduino;speed;200\n");
    havePot[pince] = true;
}

void TCPServer::askArduinoPos() {
    while (!this->_shouldStop) {
        this->broadcastMessage("strat;arduino;get pos;1\n");
        usleep(500'000);
    }
}

void TCPServer::sendToClient(const char *message, int clientSocket) {
    for (int client : clientSockets) {
        if (client == clientSocket) {
            send(client, message, strlen(message), 0);
            break;
        }
    }
}

void TCPServer::sendToClient(std::string &message, int clientSocket) {
    this->sendToClient(message.c_str(), clientSocket);
}

bool TCPServer::shouldStop() const {
    return _shouldStop;
}