// --------------------------------------------------------------------------------------
// Project: MicroManipulatorStepper
// License: MIT (see LICENSE file for full description)
//          All text in here must be included in any redistribution.
// Author:  M. S. (diffraction limited)
// --------------------------------------------------------------------------------------

#include "compat/LittleFS_compat.h"
#include <string>

#include "logging.h"
#include "utilities.h"

//*** FUNCTIONS *************************************************************************

std::vector<std::string> get_file_list(const char* dirname, bool include_dirs) {
  std::vector<std::string> file_list;

  File root = LittleFS.open(dirname, "r");
  if (!root || !root.isDirectory()) {
    LOG_ERROR("Failed to open directory %s", dirname);
    return file_list; // return empty vector
  }

  File file = root.openNextFile();
  while (file) {
    if (!file.isDirectory()) {
      file_list.emplace_back(file.name());
    } else if(include_dirs) {
      file_list.emplace_back(std::string("DIR ") + file.name());
    }
    file = root.openNextFile();
  }

  return file_list;
}