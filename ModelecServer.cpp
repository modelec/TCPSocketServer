#include "ModelecServer.h"

ModelecServer::ModelecServer(int port) : TCPServer(port)
{

}

void ModelecServer::handleMessage(const std::string& message, int clientSocket)
{
    std::cout << "REceived message" << std::endl;
    /*if (startWith(message, "request robotPose"))
    {
        this->broadcastMessage("robotPos 25 24 23 2");
    }
    else if (startWith(message, "pong"))
    {
        std::string name = split(message, " ")[1];
        std::cout << "Pong from " << name << std::endl;
    } else
    {
        std::cout << message << std::endl;
    }*/
}
