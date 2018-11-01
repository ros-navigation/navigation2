#ifndef SIMPLE_STRING_HPP
#define SIMPLE_STRING_HPP

#include <string>
#include <cstring>

namespace SafeAny
{
// Version of string that uses only two words. Good for small object optimization in linb::any
class SimpleString
{
  public:
    SimpleString(const std::string& str) : SimpleString(str.data(), str.size())
    {
    }
    SimpleString(const char* data) : SimpleString(data, strlen(data))
    {
    }

    SimpleString(const char* data, std::size_t size) : _size(size)
    {
        _data = new char[_size + 1];
        strncpy(_data, data, _size);
        _data[_size] = '\0';
    }

    SimpleString(const SimpleString& other) : SimpleString(other.data(), other.size())
    {
    }

    ~SimpleString()
    {
        if (_data)
        {
            delete[] _data;
        }
    }

    std::string toStdString() const
    {
        return std::string(_data, _size);
    }

    const char* data() const
    {
        return _data;
    }

    std::size_t size() const
    {
        return _size;
    }

  private:
    char* _data;
    std::size_t _size;
};
}

#endif   // SIMPLE_STRING_HPP
