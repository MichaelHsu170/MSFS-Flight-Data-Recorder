#pragma once

#include <string>

// Where the app keeps its files (settings.ini, flight_data.db, the debug
// log): the working directory in Debug builds, so each project checkout is
// self-contained, and the executable's folder in Release builds, so the files
// follow the installation. Returns file_name's full path there, as UTF-8.
std::string app_file_path(const char* file_name);
