#pragma once

#include <QString>

class QWidget;

// The dialogs around a KML export, shared by the map's and Trip History's
// "Export to KML" menu items so both ask and report the same way.

// Asks where to save the KML file, suggesting baseName + ".kml". Returns the
// chosen path, or an empty string if the user cancelled.
QString askKmlSaveFileName(QWidget* parent, const QString& baseName);

// Shows the error box for an export to fileName that failed with reason.
void showKmlExportFailed(QWidget* parent, const QString& fileName, const QString& reason);
