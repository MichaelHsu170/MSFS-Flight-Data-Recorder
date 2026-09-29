# Automated tests: each tst_*.cpp is its own Qt Test executable, linked
# against fdr_core (the app's code) and fake_simconnect.cpp in place of
# SimConnect.lib, so they run without MSFS. Run them with the Ctrl+Shift+B
# "Test" task, or: .vscode\scripts\build.bat Debug test
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Test)

add_library(fdr_test_support OBJECT fake_simconnect.cpp test_support.cpp)
target_link_libraries(fdr_test_support PUBLIC fdr_core Qt${QT_VERSION_MAJOR}::Test)

# Kept out of the app's bin/ directory: in Release builds the app code puts
# flight_data.db and settings.ini next to the running executable, and the
# tests create and delete those files.
set(FDR_TEST_OUTPUT_DIR "${PROJECT_BINARY_DIR}/tests")

function(fdr_add_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE fdr_test_support fdr_core Qt${QT_VERSION_MAJOR}::Test)
    # $<1:...> stops multi-config generators from appending a per-config subdirectory.
    set_target_properties(${name} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "$<1:${FDR_TEST_OUTPUT_DIR}>")
    if(EXISTS "${SQLITE3_DIR}/sqlite3.dll")
        add_custom_command(TARGET ${name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SQLITE3_DIR}/sqlite3.dll" "$<TARGET_FILE_DIR:${name}>")
    endif()
    add_test(NAME ${name} COMMAND ${name} WORKING_DIRECTORY "${FDR_TEST_OUTPUT_DIR}")
    # offscreen: widget tests need no visible desktop. The PATH prepend lets
    # the executables find Qt's DLLs without running windeployqt on them.
    # RESOURCE_LOCK: in Release builds every test uses the same
    # flight_data.db/settings.ini next to the executables, so none may run in
    # parallel with another (ctest -j).
    set_tests_properties(${name} PROPERTIES
        RESOURCE_LOCK fdr_data_files
        ENVIRONMENT "QT_QPA_PLATFORM=offscreen"
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:$<TARGET_FILE_DIR:Qt${QT_VERSION_MAJOR}::Core>")
endfunction()

fdr_add_test(tst_types)
fdr_add_test(tst_recording)
fdr_add_test(tst_airport_lookup)
fdr_add_test(tst_events)
fdr_add_test(tst_database)
fdr_add_test(tst_groups)
fdr_add_test(tst_trip_dataset)
fdr_add_test(tst_kml)
fdr_add_test(tst_settings)
fdr_add_test(tst_logger)
fdr_add_test(tst_data_table_panel)
fdr_add_test(tst_trip_history)
fdr_add_test(tst_live_status_panel)
fdr_add_test(tst_manage_groups_dialog)
fdr_add_test(tst_map_bridge)
fdr_add_test(tst_runway_match)
fdr_add_test(tst_event_filter)
fdr_add_test(tst_chart_data)
# Checks the series table against the QML that declares the series.
target_compile_definitions(tst_chart_data PRIVATE CHARTS_QML="${SRC_DIR}/resources/charts_panel.qml")
fdr_add_test(tst_map_script)
