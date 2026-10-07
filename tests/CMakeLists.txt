# Automated tests: each tst_*.cpp is its own Qt Test executable, linked
# against fdr_core (the app's code) and fake_simconnect.cpp in place of
# SimConnect.lib, so they run without MSFS. Run them with the Ctrl+Shift+B
# "Test" task, or: .vscode\scripts\build.bat Debug test
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Test)

add_library(fdr_test_support OBJECT fake_simconnect.cpp test_support.cpp)
target_link_libraries(fdr_test_support PUBLIC fdr_core Qt${QT_VERSION_MAJOR}::Test)
# The test programs live in subfolders mirroring the app's source folders;
# this lets them include the shared helpers here by name.
target_include_directories(fdr_test_support PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")

# Kept out of the app's bin/ directory: in Release builds the app code puts
# flight_data.db and settings.ini next to the running executable, and the
# tests create and delete those files.
set(FDR_TEST_OUTPUT_DIR "${PROJECT_BINARY_DIR}/tests")

# dir: the subfolder holding ${name}.cpp, the same as the app source folder
# of the code it tests.
function(fdr_add_test_common dir name)
    add_executable(${name} ${dir}/${name}.cpp)
    target_link_libraries(${name} PRIVATE fdr_test_support)
    # $<1:...> stops multi-config generators from appending a per-config subdirectory.
    set_target_properties(${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "$<1:${FDR_TEST_OUTPUT_DIR}>")
    if(EXISTS "${SQLITE3_DIR}/sqlite3.dll")
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SQLITE3_DIR}/sqlite3.dll" "$<TARGET_FILE_DIR:${name}>")
    endif()
    add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY "${FDR_TEST_OUTPUT_DIR}")
    # The PATH prepend lets the executables find Qt's DLLs without running
    # windeployqt on them. RESOURCE_LOCK: in Release builds every test uses
    # the same flight_data.db/settings.ini next to the executables, so none
    # may run in parallel with another (ctest -j).
    set_tests_properties(${name} PROPERTIES
        RESOURCE_LOCK fdr_data_files
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt${QT_VERSION_MAJOR}::Core>")
endfunction()

# offscreen: widget tests need no visible desktop.
function(fdr_add_test dir name)
    fdr_add_test_common(${dir} ${name})
    set_tests_properties(${name} PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
endfunction()

# QWebEngineView (map_widget.cpp) can't load under QT_QPA_PLATFORM=offscreen --
# its GPU process hits a fatal DCHECK (!m_scopedOverlayReadAccess) trying to
# share a GL context with an offscreen-platform window -- so unlike the
# fdr_add_test() tests, the tests that create a MapWidget need the real
# platform plugin.
function(fdr_add_webengine_test dir name)
    fdr_add_test_common(${dir} ${name})
endfunction()

fdr_add_test(recording tst_types)
fdr_add_test(recording tst_recording)
fdr_add_test(recording tst_airport_lookup)
fdr_add_test(recording tst_events)
fdr_add_test(db tst_database)
fdr_add_test(db tst_db_query)
fdr_add_test(db tst_groups)
fdr_add_test(db tst_trip_dataset)
fdr_add_test(export tst_kml)
fdr_add_test(app tst_settings)
fdr_add_test(app tst_logger)
fdr_add_test(ui tst_data_table_panel)
fdr_add_test(ui tst_trip_history)
fdr_add_test(ui tst_live_status_panel)
fdr_add_test(ui tst_manage_groups_dialog)
fdr_add_test(ui/map tst_map_bridge)
fdr_add_test(recording tst_runway_match)
fdr_add_test(recording tst_airport_candidates)
fdr_add_test(recording tst_event_filter)
fdr_add_test(recording tst_engine_power)
fdr_add_test(ui/charts tst_chart_data)
# Checks the series table against the QML that declares the series.
target_compile_definitions(tst_chart_data PRIVATE CHARTS_QML="${SRC_DIR}/ui/charts/charts_panel.qml")
fdr_add_test(ui/map tst_map_script)
fdr_add_test(ui tst_splitter_utils)
fdr_add_test(ui/charts tst_charts_panel)
# QQuickPath::path(), to read which lines Qt Graphs draws. Its Qt-version
# coupling is accepted for this one test, so Qt's "private module" warning is
# turned off for this lookup.
set(QT_NO_PRIVATE_MODULE_WARNING ON)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS QuickPrivate)
unset(QT_NO_PRIVATE_MODULE_WARNING)
target_link_libraries(tst_charts_panel PRIVATE Qt${QT_VERSION_MAJOR}::QuickPrivate)
fdr_add_webengine_test(ui/map tst_map_widget)
# A real streamed AI response, recorded once from the service.
target_compile_definitions(tst_map_widget PRIVATE AI_STREAM_RESPONSE="${CMAKE_CURRENT_SOURCE_DIR}/data/ai_stream_response.json")
fdr_add_webengine_test(ui tst_trajectory_view)
fdr_add_webengine_test(ui tst_main_window)
