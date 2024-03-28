#pragma once

#include <vector>
#include <string>

bool startWith(const std::string& str, const std::string& start)
{
    return str.rfind(start, 0) == 0;
}

bool endsWith(const std::string& str, const std::string& end)
{
    if (str.length() >= end.length())
    {
        return (0 == str.compare(str.length() - end.length(), end.length(), end));
    }
    return false;
}

bool contains(const std::string& str, const std::string& sub)
{
    return str.find(sub) != std::string::npos;
}

std::vector<std::string> split(const std::string& str, const std::string& delimiter)
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