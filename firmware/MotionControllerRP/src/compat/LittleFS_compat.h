// --------------------------------------------------------------------------------------
// Minimal Arduino "LittleFS"/File API compatibility shim, backed by pico-vfs
// (https://github.com/oyama/pico-vfs), which mounts a littlefs filesystem on the
// onboard flash and exposes it through the standard POSIX file API (fopen/fread/...).
//
// Implements only the subset of the API this project uses: LittleFS.begin()/exists()/
// info(), and File open/read/write/close, plus directory iteration via
// openNextFile()/isDirectory()/name() (used by get_file_list()).
// --------------------------------------------------------------------------------------
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstddef>
#include <string>

// A single File object represents either a regular file (backed by a FILE*), an open
// directory (backed by a DIR*), or a directory entry returned by openNextFile() (which
// carries only a name/type, matching how get_file_list() uses it -- it never opens the
// entries it enumerates).
class File {
  public:
    File() = default;

    explicit operator bool() const { return valid_; }

    bool isDirectory() const { return is_dir_; }
    const char* name() const { return name_.c_str(); }

    size_t write(const uint8_t* buf, size_t size);
    size_t read(uint8_t* buf, size_t size);

    // Only valid when this File was opened as a directory (LittleFS.open(path, "r")
    // on a directory path). Returns an invalid (false) File once entries are exhausted.
    File openNextFile();

    void close();

  private:
    friend class LittleFSClass;

    static File make_file(FILE* fp, std::string name);
    static File make_dir(void* dp, std::string path);  // dp is a DIR*
    static File make_entry(bool is_dir, std::string name);

    bool valid_ = false;
    bool is_dir_ = false;
    FILE* fp_ = nullptr;
    void* dp_ = nullptr;  // DIR*
    std::string name_;
    std::string path_;  // full path, used to reopen/enumerate children
};

struct FSInfo {
  size_t totalBytes = 0;
  size_t usedBytes = 0;  // NOTE: pico-vfs does not currently expose free-space
                         // accounting, so this is always reported as 0.
};

class LittleFSClass {
  public:
    // Mounts the onboard-flash littlefs volume (via pico-vfs's fs_init()).
    bool begin();

    bool exists(const char* path);
    File open(const char* path, const char* mode);
    bool info(FSInfo& out_info);
};

extern LittleFSClass LittleFS;
