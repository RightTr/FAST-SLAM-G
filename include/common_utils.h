#include <filesystem>
#include <string>

namespace fs = std::filesystem;

inline bool create_directory(const std::string& path) {
    if (fs::exists(path)) {
        return fs::is_directory(path);
    }
    return fs::create_directories(path);
}
