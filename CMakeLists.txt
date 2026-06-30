cmake_minimum_required(VERSION 3.15)
project(MSFS-Flight-Data-Recorder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Qt (MSVC kit). Compiler/linker is still cl.exe/link.exe from VS Build Tools —
# Qt's MSVC kit doesn't replace that toolchain, it just supplies Qt's libraries
# built against it. Point CMAKE_PREFIX_PATH at the Qt MSVC install, e.g.:
#   -DCMAKE_PREFIX_PATH=C:/Qt/6.8.0/msvc2022_64
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Charts)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Charts)

# Source
set(SRC_DIR "${PROJECT_SOURCE_DIR}/MSFS-Flight-Data-Recorder")
set(SOURCES
    ${SRC_DIR}/MSFS-Flight-Data-Recorder.cpp
    ${SRC_DIR}/db.cpp
    ${SRC_DIR}/recorder.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts)

# SQLite3 bundled in the repo
set(SQLITE3_DIR "${PROJECT_SOURCE_DIR}/third_party/sqlite3")
if(EXISTS "${SQLITE3_DIR}/sqlite3.lib")
    target_include_directories(${PROJECT_NAME} PRIVATE ${SQLITE3_DIR})
    target_link_directories(${PROJECT_NAME} PRIVATE ${SQLITE3_DIR})
    target_link_libraries(${PROJECT_NAME} PRIVATE sqlite3.lib)
else()
    message(WARNING "sqlite3.lib not found in ${SQLITE3_DIR}. Configure your system sqlite or provide sqlite3.lib.")
endif()

# Windows system libraries used by the original project
if (WIN32)
    target_link_libraries(${PROJECT_NAME} PRIVATE shlwapi.lib user32.lib Ws2_32.lib)
endif()

# SimConnect: allow user to set SIMCONNECT_DIR or leave unset
if(DEFINED SIMCONNECT_DIR)
    message(STATUS "Using SimConnect from: ${SIMCONNECT_DIR}")
    target_include_directories(${PROJECT_NAME} PRIVATE "${SIMCONNECT_DIR}/Include" "${SIMCONNECT_DIR}")
    if(EXISTS "${SIMCONNECT_DIR}/lib/static/SimConnect.lib" AND EXISTS "${SIMCONNECT_DIR}/Lib/SimConnect.lib")
        # Use static lib for Release, dynamic lib for Debug (static lib is Release-only CRT)
        target_link_libraries(${PROJECT_NAME} PRIVATE
            $<$<CONFIG:Release>:${SIMCONNECT_DIR}/lib/static/SimConnect.lib>
            $<$<CONFIG:Debug>:${SIMCONNECT_DIR}/Lib/SimConnect.lib>
        )
        message(STATUS "SimConnect: static for Release, dynamic for Debug")
    elseif(EXISTS "${SIMCONNECT_DIR}/lib/static/SimConnect.lib")
        target_link_libraries(${PROJECT_NAME} PRIVATE "${SIMCONNECT_DIR}/lib/static/SimConnect.lib")
    elseif(EXISTS "${SIMCONNECT_DIR}/Lib/SimConnect.lib")
        target_link_libraries(${PROJECT_NAME} PRIVATE "${SIMCONNECT_DIR}/Lib/SimConnect.lib")
    elseif(EXISTS "${SIMCONNECT_DIR}/SimConnect.lib")
        target_link_libraries(${PROJECT_NAME} PRIVATE "${SIMCONNECT_DIR}/SimConnect.lib")
    else()
        message(WARNING "SimConnect.lib not found inside ${SIMCONNECT_DIR}. You may need to set SIMCONNECT_DIR to your MSFS SDK path.")
    endif()
else()
    message(STATUS "SIMCONNECT_DIR not defined. If you need SimConnect, set -DSIMCONNECT_DIR=/path/to/SimConnect_SDK")
endif()

# Additional useful targets/variables
set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_DEBUG "${PROJECT_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_RELEASE "${PROJECT_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_MINSIZEREL "${PROJECT_BINARY_DIR}/bin"
    RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${PROJECT_BINARY_DIR}/bin"
)

# Copy sqlite3.dll to the output directory after build
if(EXISTS "${SQLITE3_DIR}/sqlite3.dll")
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${SQLITE3_DIR}/sqlite3.dll"
            "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
        COMMENT "Copying sqlite3.dll to output directory"
    )
endif()

# Copy SimConnect.dll to the output directory for Debug builds (dynamic linking)
if(DEFINED SIMCONNECT_DIR)
    if(EXISTS "${SIMCONNECT_DIR}/lib/SimConnect.dll")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E $<IF:$<CONFIG:Debug>,copy_if_different,true>
                "${SIMCONNECT_DIR}/lib/SimConnect.dll"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMENT "Copying SimConnect.dll to output directory (Debug only)"
        )
    elseif(EXISTS "${SIMCONNECT_DIR}/SimConnect.dll")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E $<IF:$<CONFIG:Debug>,copy_if_different,true>
                "${SIMCONNECT_DIR}/SimConnect.dll"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMENT "Copying SimConnect.dll to output directory (Debug only)"
        )
    endif()
endif()
