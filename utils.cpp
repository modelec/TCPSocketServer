#include "utils.h"

bool TCPUtils::startWith(const std::string& str, const std::string& start)
{
    return str.rfind(start, 0) == 0;
}

bool TCPUtils::endWith(const std::string& str, const std::string& end)
{
    if (str.length() >= end.length())
    {
        return (0 == str.compare(str.length() - end.length(), end.length(), end));
    }
    return false;
}

bool TCPUtils::contains(const std::string& str, const std::string& sub)
{
    return str.find(sub) != std::string::npos;
}

std::vector<std::string> TCPUtils::split(const std::string& str, const std::string& delimiter)
{
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delimiter, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos - prev);
        if (!token.empty()) tokens.push_back(token);
        prev = pos + delimiter.length();
    } while (pos < str.length() && prev < str.length());
    return tokens;
}


ArucoTag::ArucoTag(int id, std::string name, std::pair<float[2], float[3]> pos) : _id(id), _name(std::move(name)), _pos(pos) {}

int ArucoTag::id() const {
    return _id;
}

std::string ArucoTag::name() const {
    return _name;
}

std::pair<float[2], float[3]> ArucoTag::pos() const {
    return _pos;
}

void ArucoTag::setId(int id) {
    _id = id;
}

void ArucoTag::setName(const std::string& name) {
    _name = name;
}

void ArucoTag::setPos(float x, float y) {
    _pos.first[0] = x;
    _pos.first[1] = y;
}

void ArucoTag::setRot(float x, float y, float z) {
    _pos.second[0] = x;
    _pos.second[1] = y;
    _pos.second[2] = z;
}

ArucoTag& ArucoTag::operator=(const ArucoTag& tag) {
    _id = tag.id();
    _name = tag.name();
    _pos.first[0] = tag.pos().first[0];
    _pos.first[1] = tag.pos().first[1];
    _pos.second[0] = tag.pos().second[0];
    _pos.second[1] = tag.pos().second[1];
    _pos.second[2] = tag.pos().second[2];
    return *this;
}