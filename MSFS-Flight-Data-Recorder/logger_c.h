#pragma once
// C-compatible logging shim for Qt-free source files (db.cpp).
// Integer level values match Logger::Level in logger.h exactly:
//   0 = Fatal, 1 = Warning, 2 = Info, 3 = Profile
#ifdef __cplusplus
extern "C" {
#endif

void log_c(int level, const char* module, const char* msg);
void log_cf(int level, const char* module, const char* fmt, ...);

#ifdef __cplusplus
}
#endif
