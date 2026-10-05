#include "kml_export_dialog.h"

#include <QFileDialog>
#include <QMessageBox>

QString askKmlSaveFileName(QWidget* parent, const QString& baseName) {
	return QFileDialog::getSaveFileName(parent, QStringLiteral("Export to KML"),
		baseName + QStringLiteral(".kml"), QStringLiteral("KML File (*.kml)"));
}

void showKmlExportFailed(QWidget* parent, const QString& fileName, const QString& reason) {
	QMessageBox::critical(parent, QStringLiteral("Error"),
		QStringLiteral("Failed to export KML to %1.\n%2").arg(fileName, reason));
}
