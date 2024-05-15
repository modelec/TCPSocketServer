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

    clients.reserve(5);

    clients.emplace_back("ihm");
    clients.emplace_back("lidar");
    clients.emplace_back("arduino");
    clients.emplace_back("servo_moteur");
    clients.emplace_back("gc");
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
        std::cerr << "Invalid message format, token size : " << std::to_string(tokens.size()) << " from message : " << message << std::endl;
        return;
    }
    if (TCPUtils::contains(tokens[2], "stop proximity")) {
        std::vector<std::string> args = TCPUtils::split(tokens[3], ",");

        if (stoi(args[0]) < 300) {
            stopEmergency = true;

            this->lidarDectectionAngle = stod(args[1]) / 100;

            if (!handleEmergecnyFlag) {
                std::thread([this]() { handleEmergency(); }).detach();
            }
        }
        this->broadcastMessage(message, clientSocket);
    }
    else if (tokens[1] != "strat") {
        this->broadcastMessage(message, clientSocket);
    }
    // EMERGENCY
    else if (tokens[2] == "ready") {
        for (ClientTCP& client : clients)
        {
            if (client.name == tokens[0])
            {
                client.isReady = true;
                client.socket = clientSocket;
                if (client.name == "arduino")
                {
                    arduinoSocket = clientSocket;
                }
                else if (client.name == "lidar") {
                    this->broadcastMessage("strat;lidar;start;1\n");
                    this->broadcastMessage("strat;lidar;set beacon;0\n");
                    lidarSocket = clientSocket;
                }
                std::cout << client.socket << " | " << client.name << " is ready" << std::endl;
                break;
            }
        }
        checkIfAllClientsReady();
    }
    else if (tokens[0] == "gc") {
        if (tokens[2] == "axis") {
            std::vector<std::string> args = TCPUtils::split(tokens[3], ",");
            double value = std::stod(args[1]);
            if (value > -600 && value < 600) {
                value = 0;
            }
            if (args[0] == "0") {
                if (!handleEmergecnyFlag) {
                    int angle = static_cast<int>((value * PI / 2) / 327.67f * 100);
                    this->broadcastMessage("strat;arduino;angle;" + std::to_string(angle) + "\n");
                }
            }
            else if (args[0] == "1") {
                int speed = static_cast<int>((- value * (310 - 70) / 32767.0f) + 70);
                if (!handleEmergecnyFlag) {
                    this->broadcastMessage("strat;arduino;speed;" + std::to_string(speed) + "\n");
                }
                else {
                    if (speed > 0 && !(this->lidarDectectionAngle > PI / 2 || this->lidarDectectionAngle < 3 * PI / 2)) {
                        this->broadcastMessage("strat;arduino;speed;" + std::to_string(speed) + "\n");
                    }
                    else if (speed < 0 && !(this->lidarDectectionAngle < PI / 2 && this->lidarDectectionAngle > 3 *PI / 2)) {
                    this->broadcastMessage("strat;arduino;speed;" + std::to_string(speed) + "\n");
                    }
                }
            }
            else if (args[0] == "2") {
                int speed = static_cast<int>((value * 3.1) / 327.670f);
                this->broadcastMessage("start;arduino;rotate;" + std::to_string(speed));
            }
        }
        else if (tokens[2] == "button down") {
            if (tokens[3] == "0") {
                this->toggleBras();
            }
            else if (tokens[3] == "2") {
                this->togglePince(0);
            }
            else if (tokens[3] == "3") {
                this->togglePince(1);
            }
            else if (tokens[3] == "1") {
                this->togglePince(2);
            }
            else if (tokens[3] == "13") {
                this->togglePanel(0);
            }
            else if (tokens[3] == "14") {
                this->togglePanel(1);
            }
            else if (tokens[3] == "10") {
                this->broadcastMessage("strat;arduino;clear;1\n");
            }
            else if (tokens[3] == "9") {
                this->broadcastMessage("strat;arduino;clear;1\n");
            }
        }
        else if (tokens[2] == "button up") {

        }
        else if (tokens[2] == "trigger") {
            std::vector<std::string> args = TCPUtils::split(tokens[3], ",");

            int nb = std::stoi(args[0]);

            double percentage = std::stod(args[1]) / 327.670f;

            this->percentagePanel(nb + 6, static_cast<int>(percentage));
        }
        else if (tokens[2] == "disconnect") {
            this->broadcastMessage("strat;arduino;clear;1\n");
        }
    }
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

    // Close the server socket
    close(serverSocket);
}

TCPServer::~TCPServer() {
    this->stop();
}

size_t TCPServer::nbClients() const {
    return connectedClients;
}

void TCPServer::handleEmergency() {

    handleEmergecnyFlag = true;

    this->sendToClient("strat;arduino;clear;1\n", arduinoSocket);
    this->sendToClient("strat;arduino;clear;1\n", arduinoSocket);

    while (stopEmergency) {
        stopEmergency = false;
        usleep(300'000);
    }
    handleEmergecnyFlag = false;
}


void TCPServer::start()
{
    std::thread([this]() { acceptConnections(); }).detach();
}

void TCPServer::checkIfAllClientsReady() {
    bool allReady = true;
    for (auto&[name, socket, isReady] : clients)
    {
        if (!isReady)
        {
            allReady = false;
        }
    }

    if (allReady)
    {
        this->broadcastMessage("strat;all;ready;1\n");
    }
}

void TCPServer::toggleBras() {
    brasBaisser = !brasBaisser;
    if (brasBaisser) {
        this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
    } else {
        this->broadcastMessage("strat;servo_moteur;lever bras;1\n");
    }
}

void TCPServer::togglePince(int pince) {
    if (pinceState[pince] == NONE) {
        this->broadcastMessage("strat;servo_moteur;ouvrir pince;" + std::to_string(pince) + "\n");
        pinceState[pince] = FLOWER;
    }
    else if (pinceState[pince] != NONE) {
        this->broadcastMessage("strat;servo_moteur;fermer pince;" + std::to_string(pince) + "\n");
        pinceState[pince] = NONE;
    }
}

void TCPServer::togglePanel(int servo_moteur) {
    panneauCheck[servo_moteur] = !panneauCheck[servo_moteur];
    if (panneauCheck[servo_moteur]) {
        this->broadcastMessage("strat;servo_moteur;check panneau;" + std::to_string(servo_moteur + 6) + "\n");
    } else {
        this->broadcastMessage("strat;servo_moteur;uncheck panneau;" + std::to_string(servo_moteur + 6) + "\n");
    }
}

void TCPServer::percentagePanel(int servo_moteur, int percentage) {
    this->broadcastMessage("strat;servo_moteur;panneau;" + std::to_string(servo_moteur) + "," + std::to_string(percentage) + "\n");
}