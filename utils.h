#pragma once

#include <utility>
#include <vector>
#include <string>

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
    ArucoTag(int id, std::string  name, std::pair<float[2], float[3]> pos);

    ArucoTag() = default;

    [[nodiscard]] int id() const;

    [[nodiscard]] std::string name() const;

    [[nodiscard]] std::pair<float[2], float[3]> pos() const;

    void setId(int id);

    void setName(const std::string& name);

    void setPos(float x, float y);

    void setRot(float x, float y, float z);

    ArucoTag& operator=(const ArucoTag& tag);

private:
    int _id = 0;
    std::string _name;
    std::pair<float[2], float[3]> _pos;
};
