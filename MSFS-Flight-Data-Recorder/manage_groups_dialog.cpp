#include "manage_groups_dialog.h"
#include "db_connection.h"
#include "db_groups.h"
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
	if (DbConnection sql = DbConnection::readOnly()) {
		for (const TripGroup& group : queryAllGroups(sql.get())) {
			auto* item = new QListWidgetItem(group.name, list_);
			item->setFlags(item->flags() | Qt::ItemIsEditable);
			item->setData(Qt::UserRole, group.id);
			item->setToolTip(group.tripCount == 1
				? QStringLiteral("1 trip")
				: QStringLiteral("%1 trips").arg(group.tripCount));
			if (group.id == selectGroupId)
				list_->setCurrentItem(item);
		}
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

	int newId = 0;
	bool duplicate = false;
	{
		DbConnection sql = openForWriting(this, "create group");
		if (!sql)
			return;
		newId = insertGroup(sql.get(), name);
		if (newId == 0)
			duplicate = groupNameExists(sql.get(), name, 0);
	}
	if (newId == 0) {
		Logger::logf(Logger::Trace, "Groups", "Group creation failed for \"%s\" (%s)",
			qUtf8Printable(name), duplicate ? "duplicate name" : "insert failed");
		QMessageBox::critical(this, QStringLiteral("Error"),
			duplicate ? QStringLiteral("A group named \"%1\" already exists.").arg(name)
			          : QStringLiteral("Failed to create group."));
		return;
	}
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

	bool ok = false;
	{
		DbConnection sql = openForWriting(this, "delete group");
		if (!sql)
			return;
		ok = deleteGroup(sql.get(), groupId);
	}
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
	std::vector<int> orderedIds;
	orderedIds.reserve(list_->count());
	for (int i = 0; i < list_->count(); i++)
		orderedIds.push_back(list_->item(i)->data(Qt::UserRole).toInt());

	bool ok = false;
	{
		DbConnection sql = openForWriting(this, "reorder groups");
		if (!sql) {
			reload();
			return;
		}
		ok = reorderGroups(sql.get(), orderedIds);
	}
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

	bool ok = false;
	bool duplicate = false;
	{
		DbConnection sql = openForWriting(this, "rename group");
		if (!sql) {
			reload();
			return;
		}
		ok = renameGroup(sql.get(), groupId, newName);
		if (!ok)
			duplicate = groupNameExists(sql.get(), newName, groupId);
	}
	if (!ok) {
		Logger::logf(Logger::Trace, "Groups", "Rename of group id=%d to \"%s\" failed (%s)",
			groupId, qUtf8Printable(newName), duplicate ? "duplicate name" : "update failed");
		QMessageBox::critical(this, QStringLiteral("Error"),
			duplicate ? QStringLiteral("A group named \"%1\" already exists.").arg(newName)
			          : QStringLiteral("Failed to rename group."));
	} else {
		Logger::logf(Logger::Trace, "Groups", "Group id=%d renamed to \"%s\"", groupId, qUtf8Printable(newName));
	}
	reload();
	if (ok)
		emit groupsChanged();
}
