#include "app_paths.h"

#include <filesystem>
#include <Windows.h>

std::string app_file_path(const char* file_name) {
#ifdef _DEBUG
	const std::filesystem::path dir = std::filesystem::current_path();
#else
	wchar_t exe[MAX_PATH];
	GetModuleFileNameW(NULL, exe, MAX_PATH);
	const std::filesystem::path dir = std::filesystem::path(exe).parent_path();
#endif
	return (dir / file_name).u8string();
}
