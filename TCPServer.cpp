#include "TCPServer.h"

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
                std::cout << "Client requested to quit. Closing connection." << std::endl;
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

    clients.reserve(7);
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
    while (!shouldStop) {
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
    if (tokens[2] == "spawn") {
        // TODO change that to handle spawn point
        const std::string toSend = "ihm;arduino;set;500,500,-1.57079";
        this->broadcastMessage(toSend.c_str(), clientSocket);
    }
    if (tokens[2] == "start")
    {
        this->broadcastMessage(message.c_str(), clientSocket);
        this->actionNb = 0;
        this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
        this->broadcastMessage("strat;servo_moteur;ouvrir pince;1\n");
        this->broadcastMessage("strat;aruco;get aruco;1\n");
    }
    if (tokens[0] == "aruco" && tokens[2] == "get aruco") {
        std::string aruco = tokens[3];
        if (aruco == "404") {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            this->broadcastMessage("strat;aruco;get aruco;1\n");
        } else {
            std::cout << "Aruco tags received" << std::endl;
            std::vector<std::string> arucoArgs = split(aruco, ",");
            int x = static_cast<int>(std::stof(arucoArgs[2]));
            int y = static_cast<int>(std::stof(arucoArgs[4]));
            std::string toSend = "strat;arduino;go;" + std::to_string(x) + "," + std::to_string(y);
            this->broadcastMessage(toSend.c_str());
            usleep(3'000'000);
            this->broadcastMessage("strat;servo_moteur;fermer pince;1\n");
            this->broadcastMessage("strat;servo_moteur;lever bras;1\n");
            usleep(1'000'000);
            this->broadcastMessage("strat;arduino;go;1000,1500\n");
            usleep(6'000'000);
            this->broadcastMessage("strat;servo_moteur;baisser bras;1\n");
            this->broadcastMessage("strat;servo_moteur;ouvrir pince;1\n");
            this->broadcastMessage("strat;arduino;go;500,500\n");
        }
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

void TCPServer::clientDisconnected(const int clientSocket) {
    // Remove the disconnected client's socket
    clientSockets.erase(std::remove(clientSockets.begin(), clientSockets.end(), clientSocket), clientSockets.end());
    // Decrement the count of connected clients
    connectedClients--;
}

void TCPServer::stop() {
    shouldStop = true;
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
}

void TCPServer::checkIfAllClientsReady()
{
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
        this->broadcastMessage("strat;all;ready;1");
    }
}
