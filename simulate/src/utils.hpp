#pragma once

#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <string>

namespace utils {

template <typename T>
T getenv(const char* name)
{
    const char* val = std::getenv(name);
    if (!val)
        throw std::runtime_error(std::string("environment variable not set: ") + name);
    std::istringstream ss(val);
    T result;
    if (!(ss >> result))
        throw std::runtime_error(std::string("failed to parse environment variable: ") + name);
    return result;
}

template <>
inline std::string getenv<std::string>(const char* name)
{
    const char* val = std::getenv(name);
    if (!val)
        throw std::runtime_error(std::string("environment variable not set: ") + name);
    return val;
}

template <typename T>
T getenv(const char* name, const T& default_value)
{
    const char* val = std::getenv(name);
    if (!val)
        return default_value;
    std::istringstream ss(val);
    T result;
    if (!(ss >> result))
        return default_value;
    return result;
}

template <>
inline std::string getenv<std::string>(const char* name, const std::string& default_value)
{
    const char* val = std::getenv(name);
    if (!val)
        return default_value;
    return val;
}

} // namespace utils
