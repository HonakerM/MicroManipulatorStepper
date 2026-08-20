#include "LittleFS_compat.h"

#include <cstring>
#include <sys/stat.h>
#include <dirent.h>

#include "filesystem/vfs.h"

LittleFSClass LittleFS;

//--- File --------------------------------------------------------------------

File File::make_file(FILE* fp, std::string name) {
  File f;
  f.valid_ = (fp != nullptr);
  f.is_dir_ = false;
  f.fp_ = fp;
  f.name_ = std::move(name);
  return f;
}

File File::make_dir(void* dp, std::string path) {
  File f;
  f.valid_ = (dp != nullptr);
  f.is_dir_ = true;
  f.dp_ = dp;
  f.path_ = path;
  f.name_ = std::move(path);
  return f;
}

File File::make_entry(bool is_dir, std::string name) {
  File f;
  f.valid_ = true;
  f.is_dir_ = is_dir;
  f.name_ = std::move(name);
  return f;
}

size_t File::write(const uint8_t* buf, size_t size) {
  if (!valid_ || !fp_) return 0;
  return fwrite(buf, 1, size, fp_);
}

size_t File::read(uint8_t* buf, size_t size) {
  if (!valid_ || !fp_) return 0;
  return fread(buf, 1, size, fp_);
}

File File::openNextFile() {
  if (!valid_ || !is_dir_ || !dp_) return File();

  struct dirent* ent = readdir(reinterpret_cast<DIR*>(dp_));
  if (!ent) return File();

  bool is_dir = (ent->d_type == DT_DIR);
  return File::make_entry(is_dir, ent->d_name);
}

void File::close() {
  if (fp_) {
    fclose(fp_);
    fp_ = nullptr;
  }
  if (dp_) {
    closedir(reinterpret_cast<DIR*>(dp_));
    dp_ = nullptr;
  }
  valid_ = false;
}

//--- LittleFSClass -------------------------------------------------------------

bool LittleFSClass::begin() {
  return fs_init();
}

bool LittleFSClass::exists(const char* path) {
  struct stat st;
  return stat(path, &st) == 0;
}

File LittleFSClass::open(const char* path, const char* mode) {
  struct stat st;
  bool path_is_dir = (stat(path, &st) == 0) && S_ISDIR(st.st_mode);

  if (path_is_dir) {
    DIR* d = opendir(path);
    return File::make_dir(d, path);
  }

  FILE* fp = fopen(path, mode);
  return File::make_file(fp, path);
}

bool LittleFSClass::info(FSInfo& out_info) {
  out_info.totalBytes = PICO_FS_DEFAULT_SIZE;
  out_info.usedBytes = 0;
  return true;
}
