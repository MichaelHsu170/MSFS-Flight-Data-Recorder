#include "manage_groups_dialog.h"
#include "db_groups.h"
#include "db.h"
#include "logger.h"

#include <QDialogButtonBox>
#include <QDropEvent>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QVBoxLayout>

#include "sqlite3.h"

ReorderableListWidget::ReorderableListWidget(QWidget* parent) : QListWidget(parent) {
	setDragDropMode(QAbstractItemView::InternalMove);
}

void ReorderableListWidget::dropEvent(QDropEvent* event) {
	QListWidget::dropEvent(event);
	emit reordered();
}

ManageGroupsDialog::ManageGroupsDialog(QWidget* parent) : QDialog(parent) {
	setWindowTitle(QStringLiteral("Manage Trip Groups"));

	list_ = new ReorderableListWidget(this);
	list_->setSelectionMode(QAbstractItemView::SingleSelection);
	connect(list_, &QListWidget::itemChanged, this, &ManageGroupsDialog::onItemChanged);
	connect(list_, &ReorderableListWidget::reordered, this, &ManageGroupsDialog::onListReordered);

	auto* addButton = new QPushButton(QStringLiteral("New Group…"), this);
	auto* deleteButton = new QPushButton(QStringLiteral("Delete"), this);
	connect(addButton, &QPushButton::clicked, this, &ManageGroupsDialog::addGroup);
	connect(deleteButton, &QPushButton::clicked, this, &ManageGroupsDialog::deleteSelectedGroup);

	auto* buttonRow = new QHBoxLayout();
	buttonRow->addWidget(addButton);
	buttonRow->addWidget(deleteButton);
	buttonRow->addStretch();

	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, this);
	connect(buttons, &QDialogButtonBox::rejected, this, &ManageGroupsDialog::close);

	auto* layout = new QVBoxLayout(this);
	layout->addWidget(new QLabel(QStringLiteral("Double-click a group to rename it. Drag to reorder."), this));
	layout->addWidget(list_);
	layout->addLayout(buttonRow);
	layout->addWidget(buttons);
	setMinimumSize(360, 420);

	reload();
}

void ManageGroupsDialog::reload(int selectGroupId) {
	if (selectGroupId < 0 && list_->currentItem())
		selectGroupId = list_->currentItem()->data(Qt::UserRole).toInt();
	updating_ = true;
	list_->clear();
	sqlite3* sql = connect_db_readonly();
	if (sql) {
		for (const TripGroup& group : queryAllGroups(sql)) {
			auto* item = new QListWidgetItem(group.name, list_);
			item->setFlags(item->flags() | Qt::ItemIsEditable);
			item->setData(Qt::UserRole, group.id);
			item->setToolTip(group.tripCount == 1
				? QStringLiteral("1 trip")
				: QStringLiteral("%1 trips").arg(group.tripCount));
			if (group.id == selectGroupId)
				list_->setCurrentItem(item);
		}
		sqlite3_close(sql);
	}
	updating_ = false;
}

void ManageGroupsDialog::addGroup() {
	bool ok = false;
	QString name = QInputDialog::getText(this, QStringLiteral("New Group"),
		QStringLiteral("Group name:"), QLineEdit::Normal, QString(), &ok).trimmed();
	if (!ok || name.isEmpty()) {
		Logger::log(Logger::Trace, "Groups", QStringLiteral("New Group dialog cancelled or left blank"));
		return;
	}

	sqlite3* sql = connect_db_readwrite();
	if (!sql) {
		Logger::log(Logger::Trace, "Groups", QStringLiteral("Cannot create group: failed to open database for writing"));
		QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Could not open the database for writing."));
		return;
	}
	int newId = insertGroup(sql, name);
	if (newId == 0) {
		bool duplicate = false;
		for (const TripGroup& group : queryAllGroups(sql))
			duplicate = duplicate || group.name.compare(name, Qt::CaseInsensitive) == 0;
		sqlite3_close(sql);
		Logger::logf(Logger::Trace, "Groups", "Group creation failed for \"%s\" (%s)",
			qUtf8Printable(name), duplicate ? "duplicate name" : "insert failed");
		QMessageBox::critical(this, QStringLiteral("Error"),
			duplicate ? QStringLiteral("A group named \"%1\" already exists.").arg(name)
			          : QStringLiteral("Failed to create group."));
		return;
	}
	sqlite3_close(sql);
	Logger::logf(Logger::Trace, "Groups", "Group \"%s\" created (id=%d)", qUtf8Printable(name), newId);
	reload(newId);
	emit groupsChanged();
}

void ManageGroupsDialog::deleteSelectedGroup() {
	QListWidgetItem* item = list_->currentItem();
	if (!item)
		return;
	int groupId = item->data(Qt::UserRole).toInt();
	QString name = item->text();

	QMessageBox confirm(this);
	confirm.setWindowTitle(QStringLiteral("Delete Group"));
	confirm.setText(QStringLiteral("Delete the group \"%1\"?").arg(name));
	confirm.setInformativeText(QStringLiteral("Trips in this group will become Ungrouped. This does not delete any trip data."));
	confirm.setStandardButtons(QMessageBox::Yes | QMessageBox::Cancel);
	confirm.setDefaultButton(QMessageBox::Cancel);
	confirm.setIcon(QMessageBox::Warning);
	if (confirm.exec() != QMessageBox::Yes) {
		Logger::logf(Logger::Trace, "Groups", "Deletion of group \"%s\" cancelled by user", qUtf8Printable(name));
		return;
	}

	sqlite3* sql = connect_db_readwrite();
	if (!sql) {
		Logger::log(Logger::Trace, "Groups", QStringLiteral("Cannot delete group: failed to open database for writing"));
		QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Could not open the database for writing."));
		return;
	}
	bool ok = deleteGroup(sql, groupId);
	sqlite3_close(sql);
	if (!ok) {
		Logger::logf(Logger::Trace, "Groups", "Failed to delete group \"%s\" (id=%d)", qUtf8Printable(name), groupId);
		QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Failed to delete group."));
		return;
	}
	Logger::logf(Logger::Trace, "Groups", "Group \"%s\" (id=%d) deleted", qUtf8Printable(name), groupId);
	reload();
	emit groupsChanged();
}

void ManageGroupsDialog::onListReordered() {
	// reload()'s own clear()+re-populate never goes through
	// ReorderableListWidget::dropEvent(), so this guard is only a defensive
	// mirror of onItemChanged's -- kept for the same reason: cheap insurance
	// against ever reacting to reload()'s own writes.
	if (updating_)
		return;

	std::vector<int> orderedIds;
	orderedIds.reserve(list_->count());
	for (int i = 0; i < list_->count(); i++)
		orderedIds.push_back(list_->item(i)->data(Qt::UserRole).toInt());

	sqlite3* sql = connect_db_readwrite();
	if (!sql) {
		Logger::log(Logger::Trace, "Groups", QStringLiteral("Cannot reorder groups: failed to open database for writing"));
		QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Could not open the database for writing."));
		reload();
		return;
	}
	bool ok = reorderGroups(sql, orderedIds);
	sqlite3_close(sql);
	if (!ok) {
		Logger::log(Logger::Trace, "Groups", QStringLiteral("Failed to persist the new group order"));
		QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Failed to save the new group order."));
	} else {
		Logger::log(Logger::Trace, "Groups", QStringLiteral("Group order updated"));
	}
	reload();
	if (ok)
		emit groupsChanged();
}

void ManageGroupsDialog::onItemChanged(QListWidgetItem* item) {
	if (updating_)
		return;

	QString newName = item->text().trimmed();
	int groupId = item->data(Qt::UserRole).toInt();
	if (newName.isEmpty()) {
		// Revert rather than allow a blank group name.
		Logger::logf(Logger::Trace, "Groups", "Rename of group id=%d rejected: blank name; reverting", groupId);
		reload();
		return;
	}

	sqlite3* sql = connect_db_readwrite();
	if (!sql) {
		Logger::log(Logger::Trace, "Groups", QStringLiteral("Cannot rename group: failed to open database for writing"));
		QMessageBox::critical(this, QStringLiteral("Error"), QStringLiteral("Could not open the database for writing."));
		reload();
		return;
	}
	bool ok = renameGroup(sql, groupId, newName);
	if (!ok) {
		bool duplicate = false;
		for (const TripGroup& group : queryAllGroups(sql))
			duplicate = duplicate || (group.id != groupId && group.name.compare(newName, Qt::CaseInsensitive) == 0);
		Logger::logf(Logger::Trace, "Groups", "Rename of group id=%d to \"%s\" failed (%s)",
			groupId, qUtf8Printable(newName), duplicate ? "duplicate name" : "update failed");
		QMessageBox::critical(this, QStringLiteral("Error"),
			duplicate ? QStringLiteral("A group named \"%1\" already exists.").arg(newName)
			          : QStringLiteral("Failed to rename group."));
	} else {
		Logger::logf(Logger::Trace, "Groups", "Group id=%d renamed to \"%s\"", groupId, qUtf8Printable(newName));
	}
	sqlite3_close(sql);
	reload();
	if (ok)
		emit groupsChanged();
}
