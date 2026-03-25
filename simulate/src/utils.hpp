#pragma once

#include <algorithm>
#include <cstdlib>
#include <sstream>
#include <stdexcept>
#include <string>

namespace utils {

namespace detail {
inline bool parse_bool(const char* val)
{
    std::string s(val);
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    if (s == "1" || s == "true") return true;
    if (s == "0" || s == "false") return false;
    throw std::runtime_error(std::string("invalid boolean value: ") + val);
}
} // namespace detail

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
inline bool getenv<bool>(const char* name)
{
    const char* val = std::getenv(name);
    if (!val)
        throw std::runtime_error(std::string("environment variable not set: ") + name);
    return detail::parse_bool(val);
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
inline bool getenv<bool>(const char* name, const bool& default_value)
{
    const char* val = std::getenv(name);
    if (!val)
        return default_value;
    try { return detail::parse_bool(val); }
    catch (...) { return default_value; }
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
