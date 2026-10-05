#include "kml_export_dialog.h"
#include "db_connection.h"
#include "db_history.h"
#include "kml_export.h"

#include <QFileDialog>
#include <QFutureWatcher>
#include <QMessageBox>
#include <QtConcurrent/QtConcurrentRun>

void showKmlExportFailed(QWidget* parent, const QString& fileName, const QString& reason) {
	QMessageBox::critical(parent, QStringLiteral("Error"),
		QStringLiteral("Failed to export KML to %1.\n%2").arg(fileName, reason));
}

void exportTripToKml(QWidget* parent, const QString& baseName, int tripId) {
	const QString fileName = QFileDialog::getSaveFileName(parent, QStringLiteral("Export to KML"),
		baseName + QStringLiteral(".kml"), QStringLiteral("KML File (*.kml)"));
	if (fileName.isEmpty())
		return;

	auto* watcher = new QFutureWatcher<QString>(parent);
	QObject::connect(watcher, &QFutureWatcher<QString>::finished, parent, [parent, watcher, fileName]() {
		const QString error = watcher->result();
		watcher->deleteLater();
		if (!error.isEmpty())
			showKmlExportFailed(parent, fileName, error);
	});
	watcher->setFuture(QtConcurrent::run([tripId, fileName]() -> QString {
		DbConnection sql = openForReading(QStringLiteral("export trip %1 to KML").arg(tripId));
		if (!sql)
			return QStringLiteral("Could not open the trip database.");
		// The KML has no use for the aircraft title or departure time.
		TripDataset dataset = tripSamples(sql.get(), tripId, QString(), QString());
		// None recorded, or the read failed: there's no track to export.
		if (dataset.points.empty())
			return QStringLiteral("No recorded samples of this trip could be read.");
		completeTripDataset(dataset, queryLiftoffs(sql.get(), tripId), queryTouchdowns(sql.get(), tripId), queryEvents(sql.get(), tripId));
		QString error;
		exportTripDatasetToKmlFile(dataset, fileName, &error);
		return error;
	}));
}
