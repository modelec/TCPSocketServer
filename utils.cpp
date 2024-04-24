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


ArucoTag::ArucoTag(int id, std::string name, std::array<float, 2> pos, std::array<float, 3> rot) : _id(id), _name(std::move(name)), _pos(pos), _rot(rot) {}

int ArucoTag::id() const {
    return _id;
}

std::string ArucoTag::name() const {
    return _name;
}

std::array<float, 2> ArucoTag::pos() const {
    return _pos;
}

std::array<float, 3> ArucoTag::rot() const {
    return _rot;
}

void ArucoTag::setId(int id) {
    _id = id;
}

void ArucoTag::setName(const std::string& name) {
    _name = name;
}

void ArucoTag::setPos(float x, float y) {
    this->_pos[0] = x;
    this->_pos[1] = y;
}

void ArucoTag::setRot(float x, float y, float z) {
    this->_rot[0] = x;
    this->_rot[1] = y;
    this->_rot[2] = z;
}

ArucoTag& ArucoTag::operator=(const ArucoTag& tag) {
    _id = tag.id();
    _name = tag.name();
    _pos = tag.pos();
    _rot = tag.rot();
    return *this;
}

void ArucoTag::find() {
    nbFind++;
}

int ArucoTag::getNbFind() const {
    return nbFind;
}
