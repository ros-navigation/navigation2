#include <string>
#include <mutex>
#include <dlfcn.h>
#include "behavior_tree_core/shared_library.h"

namespace BT
{
SharedLibrary::SharedLibrary()
{
    _handle = nullptr;
}

void SharedLibrary::load(const std::string& path, int)
{
    std::unique_lock<std::mutex> lock(_mutex);

    if (_handle)
    {
        throw std::runtime_error("Library already loaded: " + path);
    }

    _handle = dlopen(path.c_str(), RTLD_NOW | RTLD_GLOBAL);
    if (!_handle)
    {
        const char* err = dlerror();
        throw std::runtime_error("Could not load library: " + (err ? std::string(err) : path));
    }
    _path = path;
}

void SharedLibrary::unload()
{
    std::unique_lock<std::mutex> lock(_mutex);

    if (_handle)
    {
        dlclose(_handle);
        _handle = nullptr;
    }
}

bool SharedLibrary::isLoaded() const
{
    return _handle != nullptr;
}

void* SharedLibrary::findSymbol(const std::string& name)
{
    std::unique_lock<std::mutex> lock(_mutex);

    void* result = nullptr;
    if (_handle)
    {
        result = dlsym(_handle, name.c_str());
    }
    return result;
}

const std::string& SharedLibrary::getPath() const
{
    return _path;
}

std::string SharedLibrary::prefix()
{
#if BT_OS == BT_OS_CYGWIN
    return "cyg";
#else
    return "lib";
#endif
}

std::string SharedLibrary::suffix()
{
#if BT_OS == BT_OS_MAC_OS_X
#if defined(_DEBUG) && !defined(CL_NO_SHARED_LIBRARY_DEBUG_SUFFIX)
    return "d.dylib";
#else
    return ".dylib";
#endif
#elif BT_OS == BT_OS_HPUX
#if defined(_DEBUG) && !defined(CL_NO_SHARED_LIBRARY_DEBUG_SUFFIX)
    return "d.sl";
#else
    return ".sl";
#endif
#elif BT_OS == BT_OS_CYGWIN
#if defined(_DEBUG) && !defined(CL_NO_SHARED_LIBRARY_DEBUG_SUFFIX)
    return "d.dll";
#else
    return ".dll";
#endif
#else
#if defined(_DEBUG) && !defined(CL_NO_SHARED_LIBRARY_DEBUG_SUFFIX)
    return "d.so";
#else
    return ".so";
#endif
#endif
}

}   // namespace
