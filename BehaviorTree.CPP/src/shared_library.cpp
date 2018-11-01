#include "behavior_tree_core/shared_library.h"

BT::SharedLibrary::SharedLibrary(const std::string& path, int flags)
{
    load(path, flags);
}

void* BT::SharedLibrary::getSymbol(const std::string& name)
{
    void* result = findSymbol(name);
    if (result)
        return result;
    else
        throw std::runtime_error(name);
}

bool BT::SharedLibrary::hasSymbol(const std::string& name)
{
    return findSymbol(name) != 0;
}

std::string BT::SharedLibrary::getOSName(const std::string& name)
{
    return prefix() + name + suffix();
}
