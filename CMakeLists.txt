cmake_minimum_required(VERSION 3.17)
project(MSFS-Flight-Data-Recorder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Qt (MSVC kit). Compiler/linker is still cl.exe/link.exe from VS Build Tools —
# Qt's MSVC kit doesn't replace that toolchain, it just supplies Qt's libraries
# built against it. Point CMAKE_PREFIX_PATH at the Qt MSVC install, e.g.:
#   -DCMAKE_PREFIX_PATH=C:/Qt/6.11.1/msvc2022_64
# Quick/QuickWidgets: Qt Graphs' 2D chart surface (GraphsView) has no public
# C++/QWidget header in this Qt version, only QML -- see charts_panel.h --
# so charts_panel.qml is hosted via QQuickWidget.
# WebEngineWidgets/WebChannel: the trajectory map embeds Leaflet/OSM (and,
# from step 6, Google Maps) via QWebEngineView, bridged to C++ over
# QWebChannel -- see map_widget.h.
# CoreTools: pulls in the Qt6::windeployqt imported target used below to copy
# Qt's runtime (DLLs, QML modules, WebEngine's process/resources/translations)
# next to the exe -- needed for VS Code's F5 launch, which doesn't inherit
# Qt's bin directory on PATH the way a manual terminal build might.
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets Graphs GraphsWidgets Concurrent Quick QuickWidgets QuickControls2 WebEngineWidgets WebChannel CoreTools)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets Graphs GraphsWidgets Concurrent Quick QuickWidgets QuickControls2 WebEngineWidgets WebChannel CoreTools)

# Source
set(SRC_DIR "${PROJECT_SOURCE_DIR}/MSFS-Flight-Data-Recorder")
set(SOURCES
    ${SRC_DIR}/main.cpp
    ${SRC_DIR}/db.cpp
    ${SRC_DIR}/recorder.cpp
    ${SRC_DIR}/recorder_bridge.cpp
    ${SRC_DIR}/live_status_panel.cpp
    ${SRC_DIR}/db_history.cpp
    ${SRC_DIR}/trip_history_panel.cpp
    ${SRC_DIR}/charts_panel.cpp
    ${SRC_DIR}/data_table_panel.cpp
    ${SRC_DIR}/map_bridge.cpp
    ${SRC_DIR}/map_widget.cpp
    ${SRC_DIR}/trajectory_view.cpp
    ${SRC_DIR}/main_window.cpp
    ${SRC_DIR}/app_settings.cpp
    ${SRC_DIR}/logger.cpp
    ${SRC_DIR}/resources/charts.qrc
    ${SRC_DIR}/resources/map.qrc
    ${SRC_DIR}/resources/app.qrc
    ${SRC_DIR}/resources/app.rc
)

# rc.exe searches the directory of the .rc file itself, but passing the
# include dir explicitly ensures it always finds app_icon.ico regardless of
# how CMake invokes the resource compiler.
if(MSVC)
    set_source_files_properties(
        ${SRC_DIR}/resources/app.rc
        PROPERTIES COMPILE_FLAGS "/I \"${SRC_DIR}/resources\""
    )
endif()

add_executable(${PROJECT_NAME} WIN32 ${SOURCES})

target_link_libraries(${PROJECT_NAME} PRIVATE Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Graphs Qt${QT_VERSION_MAJOR}::GraphsWidgets Qt${QT_VERSION_MAJOR}::Concurrent Qt${QT_VERSION_MAJOR}::Quick Qt${QT_VERSION_MAJOR}::QuickWidgets Qt${QT_VERSION_MAJOR}::QuickControls2 Qt${QT_VERSION_MAJOR}::WebEngineWidgets Qt${QT_VERSION_MAJOR}::WebChannel)

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
    # Always link dynamically so both Debug and Release deploy the same SimConnect.dll.
    # The static lib is built with the Release CRT and causes CRT mismatch in Debug.
    if(EXISTS "${SIMCONNECT_DIR}/Lib/SimConnect.lib")
        target_link_libraries(${PROJECT_NAME} PRIVATE "${SIMCONNECT_DIR}/Lib/SimConnect.lib")
        message(STATUS "SimConnect: dynamic linking (Lib/SimConnect.lib)")
    elseif(EXISTS "${SIMCONNECT_DIR}/SimConnect.lib")
        target_link_libraries(${PROJECT_NAME} PRIVATE "${SIMCONNECT_DIR}/SimConnect.lib")
        message(STATUS "SimConnect: dynamic linking (SimConnect.lib)")
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

# Copy SimConnect.dll to the output directory (dynamic linking for all configs)
if(DEFINED SIMCONNECT_DIR)
    if(EXISTS "${SIMCONNECT_DIR}/lib/SimConnect.dll")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${SIMCONNECT_DIR}/lib/SimConnect.dll"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMENT "Copying SimConnect.dll to output directory"
        )
    elseif(EXISTS "${SIMCONNECT_DIR}/SimConnect.dll")
        add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${SIMCONNECT_DIR}/SimConnect.dll"
                "$<TARGET_FILE_DIR:${PROJECT_NAME}>"
            COMMENT "Copying SimConnect.dll to output directory"
        )
    endif()
endif()

# Deploy Qt's own runtime next to the exe via windeployqt: DLLs, platform/
# image-format plugins, the QML modules ChartsPanel's QQuickWidget needs
# (QtGraphs, QtQuick.Layouts), and QtWebEngine's larger footprint
# (QtWebEngineProcess.exe, resources/*.pak, locale .pak files, icudtl.dat,
# v8_context_snapshot.bin). Without this the exe only runs when Qt's bin
# directory happens to be on PATH (e.g. a manual terminal build) -- it fails
# to start at all under VS Code's F5 launch otherwise.
#
# Flags used to trim the deployment:
#   --no-translations              app is English-only; no locale .qm files needed
#   --no-opengl-sw                 any MSFS-capable PC has a real GPU; skip
#                                  opengl32sw.dll software rasterizer fallback
#   --skip-plugin-types            qmltooling: QML debugger/profiler (dev tool)
#                                  generic: qtuiotouchplugin (TUIO multi-touch table)
#                                  platforminputcontexts: qtvirtualkeyboardplugin
#   --exclude-plugins              image-format plugins the app doesn't load:
#                                  gif, macOS icns, pdf, svg-as-image, tga, tiff, wbmp, webp
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND Qt${QT_VERSION_MAJOR}::windeployqt
        --qmldir "${SRC_DIR}/resources"
        --no-translations
        --no-opengl-sw
        --skip-plugin-types qmltooling,generic,platforminputcontexts
        --exclude-plugins qgif,qicns,qpdf,qsvg,qtga,qtiff,qwbmp,qwebp
        $<TARGET_FILE:${PROJECT_NAME}>
    COMMENT "Running windeployqt to copy Qt runtime DLLs/plugins"
)

# Remove Qt modules that windeployqt copies because they exist on disk but
# are not used by this app.  cmake -E rm -f silently skips missing files so
# this is safe to re-run on incremental builds.
#
# Top-level DLLs:
#   Qt6Lottie / Qt6LottieVectorImageGenerator  — Lottie animation engine
#   Qt6Pdf                                     — PDF rendering
#   Qt6SerialPort                              — serial port (SimConnect uses SDK)
# Note: Qt6Positioning and Qt6PrintSupport are kept — Qt6WebEngineCore.dll loads
# them at startup as hard runtime dependencies even though the app never calls
# their C++ APIs directly.
#   Qt6VirtualKeyboard                         — touch-screen virtual keyboard
#   Qt6QuickVectorImageGenerator               — SVG/Lottie vector rendering
#   QuickControls2 extra styles                — only Windows (default) + Basic
#                                                (fallback) are needed; the rest
#                                                are selectable themes we don't use
#
# Plugin subdirectories:
#   position/             — Qt Positioning runtime plugins
#   vectorimageformats/   — SVG/Lottie/PDF image-format plugins
#
# QML module subdirectories (unused features deployed by windeployqt's QML scan):
#   QtQuick.Controls extra styles  — same as above, mirror in qml/ tree
#   QtQuick.Dialogs                — file/color/font dialogs (app uses C++ QDialog)
#   QtQuick.LocalStorage           — QML SQL local storage API
#   QtQuick.Particles              — particle-effects system
#   QtQuick.Pdf                    — PDF viewer QML component
#   QtQuick.Timeline               — animation timeline (design tool)
#   QtQuick.VectorImage            — SVG/Lottie vector image QML type
#   QtQuick.VirtualKeyboard        — on-screen virtual keyboard
#   QtQml.StateMachine             — QML state-machine API
#   QtQml.XmlListModel             — XPath/XQuery list model
#   QtQuick3D add-ons              — 3D engine extras not used by QtGraphs 2D charts:
#                                    AssetUtils, Effects, Helpers, MaterialEditor,
#                                    ParticleEffects, Particles3D, Physics,
#                                    SpatialAudio, Xr, lightmapviewer
set(D "$<TARGET_FILE_DIR:${PROJECT_NAME}>")
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E rm -f
        "${D}/Qt6Lottie.dll"          "${D}/Qt6Lottied.dll"
        "${D}/Qt6LottieVectorImageGenerator.dll"    "${D}/Qt6LottieVectorImageGeneratord.dll"
        "${D}/Qt6Pdf.dll"             "${D}/Qt6Pdfd.dll"
        "${D}/Qt6SerialPort.dll"      "${D}/Qt6SerialPortd.dll"
        "${D}/Qt6VirtualKeyboard.dll" "${D}/Qt6VirtualKeyboardd.dll"
        "${D}/Qt6QuickVectorImageGenerator.dll"     "${D}/Qt6QuickVectorImageGeneratord.dll"
        "${D}/Qt6QuickControls2FluentWinUI3StyleImpl.dll"  "${D}/Qt6QuickControls2FluentWinUI3StyleImpld.dll"
        "${D}/Qt6QuickControls2Fusion.dll"          "${D}/Qt6QuickControls2Fusiond.dll"
        "${D}/Qt6QuickControls2FusionStyleImpl.dll" "${D}/Qt6QuickControls2FusionStyleImpld.dll"
        "${D}/Qt6QuickControls2Imagine.dll"         "${D}/Qt6QuickControls2Imagined.dll"
        "${D}/Qt6QuickControls2ImagineStyleImpl.dll" "${D}/Qt6QuickControls2ImagineStyleImpld.dll"
        "${D}/Qt6QuickControls2Material.dll"        "${D}/Qt6QuickControls2Materiald.dll"
        "${D}/Qt6QuickControls2MaterialStyleImpl.dll" "${D}/Qt6QuickControls2MaterialStyleImpld.dll"
        "${D}/Qt6QuickControls2Universal.dll"       "${D}/Qt6QuickControls2Universald.dll"
        "${D}/Qt6QuickControls2UniversalStyleImpl.dll" "${D}/Qt6QuickControls2UniversalStyleImpld.dll"
    COMMAND ${CMAKE_COMMAND} -E rm -rf
        "${D}/position"
        "${D}/vectorimageformats"
        "${D}/qml/QtQuick/Controls/FluentWinUI3"
        "${D}/qml/QtQuick/Controls/Fusion"
        "${D}/qml/QtQuick/Controls/Imagine"
        "${D}/qml/QtQuick/Controls/Material"
        "${D}/qml/QtQuick/Controls/Universal"
        "${D}/qml/QtQuick/Dialogs"
        "${D}/qml/QtQuick/LocalStorage"
        "${D}/qml/QtQuick/Particles"
        "${D}/qml/QtQuick/Pdf"
        "${D}/qml/QtQuick/Timeline"
        "${D}/qml/QtQuick/VectorImage"
        "${D}/qml/QtQuick/VirtualKeyboard"
        "${D}/qml/QtQml/StateMachine"
        "${D}/qml/QtQml/XmlListModel"
        "${D}/qml/QtQuick3D/AssetUtils"
        "${D}/qml/QtQuick3D/Effects"
        "${D}/qml/QtQuick3D/Helpers"
        "${D}/qml/QtQuick3D/MaterialEditor"
        "${D}/qml/QtQuick3D/ParticleEffects"
        "${D}/qml/QtQuick3D/Particles3D"
        "${D}/qml/QtQuick3D/Physics"
        "${D}/qml/QtQuick3D/SpatialAudio"
        "${D}/qml/QtQuick3D/Xr"
        "${D}/qml/QtQuick3D/lightmapviewer"
    COMMENT "Removing unused Qt modules from deployment"
)
