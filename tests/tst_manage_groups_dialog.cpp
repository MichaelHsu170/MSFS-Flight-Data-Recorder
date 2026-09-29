// Manage Groups dialog (manage_groups_dialog.cpp): list, add, rename,
// delete and reorder, each written straight to the database.
#include "test_support.h"

#include "db.h"
#include "db_groups.h"
#include "manage_groups_dialog.h"

#include <QInputDialog>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
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
	static QStringList dbGroups() {
		QStringList out;
		sqlite3* db = connect_db_readonly();
		for (const TripGroup& g : queryAllGroups(db))
			out << g.name;
		sqlite3_close(db);
		return out;
	}
	static int addGroup(const char* name) {
		sqlite3* db = connect_db_readwrite();
		const int id = insertGroup(db, name);
		sqlite3_close(db);
		return id;
	}

private slots:
	void initTestCase() { isolateFiles(); }
	void init() {
		removeDatabase();
		migrate_db();
	}

	void listsGroupsInOrderWithTripCounts() {
		addGroup("Training");
		addGroup("Ops");
		ManageGroupsDialog dialog;
		QCOMPARE(items(dialog), (QStringList{ "Training", "Ops" }));
		QCOMPARE(list(dialog)->item(0)->toolTip(), QStringLiteral("0 trips"));
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
			onNextModal([&message](QWidget* box) {
				message = static_cast<QMessageBox*>(box)->text();
				clickDialogButton(box, "OK");
			});
			static_cast<QInputDialog*>(input)->setTextValue("TRAINING");
			clickDialogButton(input, "OK");
		});
		button(dialog, QString::fromUtf8("New Group…"))->click();
		QCOMPARE(message, QStringLiteral("A group named \"TRAINING\" already exists."));
		QCOMPARE(dbGroups(), QStringList{ "Training" });
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

	void renameToAnExistingNameIsRejectedWithAMessage() {
		addGroup("Training");
		addGroup("Ops");
		ManageGroupsDialog dialog;
		QSignalSpy changed(&dialog, &ManageGroupsDialog::groupsChanged);
		QString message;
		onNextModal([&message](QWidget* box) {
			message = static_cast<QMessageBox*>(box)->text();
			clickDialogButton(box, "OK");
		});
		list(dialog)->item(0)->setText("OPS");
		QCOMPARE(message, QStringLiteral("A group named \"OPS\" already exists."));
		QCOMPARE(dbGroups(), (QStringList{ "Training", "Ops" }));
		QCOMPARE(items(dialog), (QStringList{ "Training", "Ops" }));
		QCOMPARE(changed.count(), 0);
	}

	void unopenableDatabaseShowsAnError() {
		ManageGroupsDialog dialog;
		removeDatabase(); // the write connection can't be opened now
		QString message;
		onNextModal([&message](QWidget* input) {
			onNextModal([&message](QWidget* box) {
				message = static_cast<QMessageBox*>(box)->text();
				clickDialogButton(box, "OK");
			});
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

	void reorderingIsSaved() {
		addGroup("A");
		addGroup("B");
		addGroup("C");
		ManageGroupsDialog dialog;
		QListWidget* l = list(dialog);
		QListWidgetItem* c = l->takeItem(2);
		l->insertItem(0, c);
		// What ReorderableListWidget::dropEvent() does after an internal move.
		QMetaObject::invokeMethod(l, "reordered");
		QCOMPARE(dbGroups(), (QStringList{ "C", "A", "B" }));
	}
};

QTEST_MAIN(TstManageGroupsDialog)
#include "tst_manage_groups_dialog.moc"
