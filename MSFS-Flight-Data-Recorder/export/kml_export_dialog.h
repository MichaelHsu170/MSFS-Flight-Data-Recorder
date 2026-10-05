#pragma once

#include <QString>

class QWidget;

// A KML export from the UI, shared by the map's and Trip History's
// "Export to KML" menu items so both ask, export and report the same way.

// Asks where to save trip tripId's KML file, suggesting baseName + ".kml",
// then loads the trip from the database and writes the file on a background
// thread, so a long trip doesn't freeze the window. Shows the error box on
// parent if the export fails, or if no sample of the trip could be read (no
// file is written then). Does nothing if the user cancels.
void exportTripToKml(QWidget* parent, const QString& baseName, int tripId);

// Shows the error box for an export to fileName that failed with reason.
void showKmlExportFailed(QWidget* parent, const QString& fileName, const QString& reason);
