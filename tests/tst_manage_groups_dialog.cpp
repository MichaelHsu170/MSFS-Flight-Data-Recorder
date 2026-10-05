// Manage Groups dialog (manage_groups_dialog.cpp): list, add, rename,
// delete and reorder, each written straight to the database, and closing.
#include "test_support.h"

#include "db.h"
#include "db_groups.h"
#include "logger.h"
#include "manage_groups_dialog.h"

#include <QDialogButtonBox>
#include <QDropEvent>
#include <QInputDialog>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QMimeData>
#include <QPushButton>
#include <QTemporaryDir>
#include <QtTest>

using namespace TestSupport;

class TstManageGroupsDialog : public QObject {
	Q_OBJECT

private:
	static QListWidget* list(ManageGroupsDialog& d) { return d.findChild<QListWidget*>(); }
	static QPushButton* button(ManageGroupsDialog& d, const QString& text) {
		for (QPushButton* b : d.findChildren<QPushButton*>())
			if (b->text() == text)
				return b;
		return nullptr;
	}
	static QStringList items(ManageGroupsDialog& d) {
		QStringList out;
		for (int i = 0; i < list(d)->count(); ++i)
			out << list(d)->item(i)->text();
		return out;
	}
	static QString hint(ManageGroupsDialog& d) { return d.findChild<QLabel*>()->text(); }
	static QStringList dbGroups() {
		QStringList out;
		sqlite3* db = connect_db_readonly();
		for (const TripGroup& g : queryAllGroups(db))
			out << g.name;
		sqlite3_close(db);
		return out;
	}
	// Stores the text of the next message box in *message and closes it.
	static void captureNextMessage(QString* message) {
		onNextModal([message](QWidget* box) {
			*message = static_cast<QMessageBox*>(box)->text();
			clickDialogButton(box, "OK");
		});
	}
	// Moves the last item to the top, as a drag would, then delivers the drop.
	// Qt only routes drop events to the target of a real drag session, so
	// dropEvent() is called directly (through a member pointer, which a
	// using-declaration makes public).
	struct DropAccess : ReorderableListWidget {
		using ReorderableListWidget::dropEvent;
	};
	static void dragLastItemToTop(ManageGroupsDialog& d) {
		auto* l = d.findChild<ReorderableListWidget*>();
		QVERIFY(l);
		l->insertItem(0, l->takeItem(l->count() - 1));
		QMimeData mime; // nothing the model accepts: the move above is the drop's effect
		QDropEvent drop(QPointF(5, 5), Qt::MoveAction, &mime, Qt::LeftButton, Qt::NoModifier);
		void (ReorderableListWidget::*deliver)(QDropEvent*) = &DropAccess::dropEvent;
		(l->*deliver)(&drop);
	}

	QTemporaryDir logDir_;
	QString logPath_;

private slots:
	// Logger::init() takes effect once per process, so it runs here.
	void initTestCase() {
		isolateFiles();
		logPath_ = logDir_.filePath(QStringLiteral("manage_groups.log"));
		Logger::init(Logger::Warning, logPath_);
	}
	void init() {
		removeDatabase();
		migrate_db();
	}
	void cleanup() { cancelPendingModals(); } // no dialog action outlives its test

	void listsGroupsInOrderWithTripCounts() {
		const int training = addGroup("Training");
		const int ops = addGroup("Ops");
		addGroup("Empty");
		addTrip(1, training);
		addTrip(2, ops);
		addTrip(3, ops);
		addTrip(4);
		ManageGroupsDialog dialog;
		QCOMPARE(items(dialog), (QStringList{ "Training", "Ops", "Empty" }));
		QCOMPARE(list(dialog)->item(0)->toolTip(), QStringLiteral("1 trip"));
		QCOMPARE(list(dialog)->item(1)->toolTip(), QStringLiteral("2 trips"));
		QCOMPARE(list(dialog)->item(2)->toolTip(), QStringLiteral("0 trips"));
		QCOMPARE(hint(dialog), QStringLiteral("Double-click a group to rename it. Drag to reorder."));
	}

	void unreadableDatabaseIsReportedInsteadOfAnEmptyList() {
		removeDatabase();
		ManageGroupsDialog dialog;
		QVERIFY(items(dialog).isEmpty());
		QCOMPARE(hint(dialog), QStringLiteral("Could not open the database, so no groups can be shown."));
		QVERIFY(warningLogged(logPath_, { QStringLiteral("Manage Groups") }));

		// Once it can be read again, the next reload lists the groups.
		migrate_db();
		addGroup("Training");
		onNextModal([](QWidget* input) {
			static_cast<QInputDialog*>(input)->setTextValue("Ops");
			clickDialogButton(input, "OK");
		});
		button(dialog, QString::fromUtf8("New Group…"))->click();
		QCOMPARE(items(dialog), (QStringList{ "Training", "Ops" }));
		QCOMPARE(hint(dialog), QStringLiteral("Double-click a group to rename it. Drag to reorder."));
	}

	void addsAGroup() {
		ManageGroupsDialog dialog;
		QSignalSpy changed(&dialog, &ManageGroupsDialog::groupsChanged);
		onNextModal([](QWidget* input) {
			static_cast<QInputDialog*>(input)->setTextValue("  New One ");
			clickDialogButton(input, "OK");
		});
		button(dialog, QString::fromUtf8("New Group…"))->click();
		QCOMPARE(dbGroups(), QStringList{ "New One" });
		QCOMPARE(items(dialog), QStringList{ "New One" });
		QCOMPARE(list(dialog)->currentItem()->text(), QStringLiteral("New One"));
		QCOMPARE(changed.count(), 1);
	}

	void duplicateNameIsRejectedWithAMessage() {
		addGroup("Training");
		ManageGroupsDialog dialog;
		QString message;
		onNextModal([&message](QWidget* input) {
			captureNextMessage(&message);
			static_cast<QInputDialog*>(input)->setTextValue("TRAINING");
			clickDialogButton(input, "OK");
		});
		button(dialog, QString::fromUtf8("New Group…"))->click();
		QCOMPARE(message, QStringLiteral("A group named \"TRAINING\" already exists."));
		QCOMPARE(dbGroups(), QStringList{ "Training" });
	}

	void failedCreateShowsAnError() {
		exec("CREATE TRIGGER block_insert BEFORE INSERT ON trip_groups BEGIN SELECT RAISE(ABORT, 'blocked'); END;");
		ManageGroupsDialog dialog;
		QSignalSpy changed(&dialog, &ManageGroupsDialog::groupsChanged);
		QString message;
		onNextModal([&message](QWidget* input) {
			captureNextMessage(&message);
			static_cast<QInputDialog*>(input)->setTextValue("New One");
			clickDialogButton(input, "OK");
		});
		button(dialog, QString::fromUtf8("New Group…"))->click();
		QCOMPARE(message, QStringLiteral("Failed to create group."));
		QVERIFY(dbGroups().isEmpty());
		QVERIFY(items(dialog).isEmpty());
		QCOMPARE(changed.count(), 0);
	}

	void cancellingAddChangesNothing() {
		ManageGroupsDialog dialog;
		onNextModal([](QWidget* input) {
			static_cast<QInputDialog*>(input)->setTextValue("Ignored");
			clickDialogButton(input, "Cancel");
		});
		button(dialog, QString::fromUtf8("New Group…"))->click();
		QVERIFY(dbGroups().isEmpty());
	}

	void editingAnItemRenamesTheGroup() {
		addGroup("Training");
		ManageGroupsDialog dialog;
		QSignalSpy changed(&dialog, &ManageGroupsDialog::groupsChanged);
		list(dialog)->item(0)->setText(" Flight Training ");
		QCOMPARE(dbGroups(), QStringList{ "Flight Training" });
		QCOMPARE(changed.count(), 1);
	}

	// The renamed group stays selected, so it can be deleted next without
	// clicking it again.
	void aRenamedGroupStaysSelected() {
		addGroup("Training");
		addGroup("Ops");
		ManageGroupsDialog dialog;
		list(dialog)->setCurrentRow(1);
		list(dialog)->item(1)->setText("Operations");
		QVERIFY(list(dialog)->currentItem());
		QCOMPARE(list(dialog)->currentItem()->text(), QStringLiteral("Operations"));
	}

	void renameToAnExistingNameIsRejectedWithAMessage() {
		addGroup("Training");
		addGroup("Ops");
		ManageGroupsDialog dialog;
		QSignalSpy changed(&dialog, &ManageGroupsDialog::groupsChanged);
		QString message;
		captureNextMessage(&message);
		list(dialog)->item(0)->setText("OPS");
		QCOMPARE(message, QStringLiteral("A group named \"OPS\" already exists."));
		QCOMPARE(dbGroups(), (QStringList{ "Training", "Ops" }));
		QCOMPARE(items(dialog), (QStringList{ "Training", "Ops" }));
		QCOMPARE(changed.count(), 0);
	}

	void failedRenameShowsAnErrorAndKeepsTheName() {
		addGroup("Training");
		exec("CREATE TRIGGER block_update BEFORE UPDATE ON trip_groups BEGIN SELECT RAISE(ABORT, 'blocked'); END;");
		ManageGroupsDialog dialog;
		QSignalSpy changed(&dialog, &ManageGroupsDialog::groupsChanged);
		QString message;
		captureNextMessage(&message);
		list(dialog)->item(0)->setText("Renamed");
		QCOMPARE(message, QStringLiteral("Failed to rename group."));
		QCOMPARE(dbGroups(), QStringList{ "Training" });
		QCOMPARE(items(dialog), QStringList{ "Training" });
		QCOMPARE(changed.count(), 0);
	}

	void unopenableDatabaseShowsAnError() {
		ManageGroupsDialog dialog;
		removeDatabase(); // the write connection can't be opened now
		QString message;
		onNextModal([&message](QWidget* input) {
			captureNextMessage(&message);
			static_cast<QInputDialog*>(input)->setTextValue("New One");
			clickDialogButton(input, "OK");
		});
		button(dialog, QString::fromUtf8("New Group…"))->click();
		QCOMPARE(message, QStringLiteral("Could not open the database for writing."));
	}

	void blankRenameIsReverted() {
		addGroup("Training");
		ManageGroupsDialog dialog;
		list(dialog)->item(0)->setText("   ");
		QCOMPARE(dbGroups(), QStringList{ "Training" });
		QCOMPARE(items(dialog), QStringList{ "Training" });
	}

	void deletingAfterConfirmation() {
		addGroup("Training");
		addGroup("Ops");
		ManageGroupsDialog dialog;
		list(dialog)->setCurrentRow(0);
		onNextModal([](QWidget* box) { clickDialogButton(box, "Cancel"); });
		button(dialog, "Delete")->click();
		QCOMPARE(dbGroups().size(), 2);
		onNextModal([](QWidget* box) { clickDialogButton(box, "Yes"); });
		button(dialog, "Delete")->click();
		QCOMPARE(dbGroups(), QStringList{ "Ops" });
		QCOMPARE(items(dialog), QStringList{ "Ops" });
	}

	void deleteWithoutASelectionDoesNothing() {
		addGroup("Training");
		ManageGroupsDialog dialog;
		QVERIFY(!list(dialog)->currentItem());
		button(dialog, "Delete")->click(); // no confirmation box would be answered
		QCOMPARE(dbGroups(), QStringList{ "Training" });
	}

	void failedDeleteShowsAnErrorAndKeepsTheGroup() {
		addGroup("Training");
		exec("CREATE TRIGGER block_delete BEFORE DELETE ON trip_groups BEGIN SELECT RAISE(ABORT, 'blocked'); END;");
		ManageGroupsDialog dialog;
		QSignalSpy changed(&dialog, &ManageGroupsDialog::groupsChanged);
		list(dialog)->setCurrentRow(0);
		QString message;
		onNextModal([&message](QWidget* box) {
			captureNextMessage(&message);
			clickDialogButton(box, "Yes");
		});
		button(dialog, "Delete")->click();
		QCOMPARE(message, QStringLiteral("Failed to delete group."));
		QCOMPARE(dbGroups(), QStringList{ "Training" });
		QCOMPARE(changed.count(), 0);
	}

	void deleteWithoutAWritableDatabaseShowsAnError() {
		addGroup("Training");
		ManageGroupsDialog dialog;
		list(dialog)->setCurrentRow(0);
		removeDatabase();
		QString message;
		onNextModal([&message](QWidget* box) {
			captureNextMessage(&message);
			clickDialogButton(box, "Yes");
		});
		button(dialog, "Delete")->click();
		QCOMPARE(message, QStringLiteral("Could not open the database for writing."));
	}

	void renameWithoutAWritableDatabaseShowsAnErrorAndReloads() {
		addGroup("Training");
		ManageGroupsDialog dialog;
		QSignalSpy changed(&dialog, &ManageGroupsDialog::groupsChanged);
		removeDatabase();
		QString message;
		captureNextMessage(&message);
		list(dialog)->item(0)->setText("Renamed");
		QCOMPARE(message, QStringLiteral("Could not open the database for writing."));
		QVERIFY(items(dialog).isEmpty()); // reloaded from the (now missing) database
		QCOMPARE(changed.count(), 0);
	}

	void droppingAnItemSavesTheNewOrder() {
		addGroup("A");
		addGroup("B");
		addGroup("C");
		ManageGroupsDialog dialog;
		QSignalSpy changed(&dialog, &ManageGroupsDialog::groupsChanged);
		dragLastItemToTop(dialog);
		QCOMPARE(dbGroups(), (QStringList{ "C", "A", "B" }));
		QCOMPARE(items(dialog), (QStringList{ "C", "A", "B" }));
		QCOMPARE(changed.count(), 1);
	}

	void failedReorderShowsAnErrorAndRestoresTheOrder() {
		addGroup("A");
		addGroup("B");
		exec("CREATE TRIGGER block_update BEFORE UPDATE ON trip_groups BEGIN SELECT RAISE(ABORT, 'blocked'); END;");
		ManageGroupsDialog dialog;
		QSignalSpy changed(&dialog, &ManageGroupsDialog::groupsChanged);
		QString message;
		captureNextMessage(&message);
		dragLastItemToTop(dialog);
		QCOMPARE(message, QStringLiteral("Failed to save the new group order."));
		QCOMPARE(dbGroups(), (QStringList{ "A", "B" }));
		QCOMPARE(items(dialog), (QStringList{ "A", "B" }));
		QCOMPARE(changed.count(), 0);
	}

	void reorderWithoutAWritableDatabaseShowsAnError() {
		addGroup("A");
		addGroup("B");
		ManageGroupsDialog dialog;
		removeDatabase();
		QString message;
		captureNextMessage(&message);
		dragLastItemToTop(dialog);
		QCOMPARE(message, QStringLiteral("Could not open the database for writing."));
		QVERIFY(items(dialog).isEmpty());
	}

	void closeButtonClosesTheDialog() {
		ManageGroupsDialog dialog;
		dialog.show();
		QVERIFY(dialog.isVisible());
		dialog.findChild<QDialogButtonBox*>()->button(QDialogButtonBox::Close)->click();
		QVERIFY(!dialog.isVisible());
	}
};

QTEST_MAIN(TstManageGroupsDialog)
#include "tst_manage_groups_dialog.moc"
