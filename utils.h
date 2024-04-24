#pragma once

#include <array>
#include <utility>
#include <vector>
#include <string>

#define PI 3.14159265358979323846

enum PinceState {
    WHITE_FLOWER,
    PURPLE_FLOWER,
    NONE
};

namespace TCPUtils {
    bool startWith(const std::string& str, const std::string& start);

    bool endWith(const std::string& str, const std::string& end);

    bool contains(const std::string& str, const std::string& sub);

    std::vector<std::string> split(const std::string& str, const std::string& delimiter);
}

class ArucoTag {

public:
    ArucoTag(int id, std::string  name, std::array<float, 2> pos, std::array<float, 3> rot);

    ArucoTag() = default;

    [[nodiscard]] int id() const;

    [[nodiscard]] std::string name() const;

    [[nodiscard]] std::array<float, 2> pos() const;

    [[nodiscard]] std::array<float, 3> rot() const;

    void setId(int id);

    void setName(const std::string& name);

    void setPos(float x, float y);

    void setRot(float x, float y, float z);

    ArucoTag& operator=(const ArucoTag& tag);

    void find();

    [[nodiscard]] int getNbFind() const;

private:
    int _id = -1;
    std::string _name;
    std::array<float, 2> _pos;
    std::array<float, 3> _rot;
    int nbFind = 0;
};

class FlowerAruco {

public:
    FlowerAruco();

    explicit FlowerAruco(ArucoTag* tag);

    [[nodiscard]] ArucoTag* getTag() const;

    [[nodiscard]] std::array<float, 2> getPos() const;

private:
    ArucoTag* tag;

    std::array<float, 2> _realPos;
};