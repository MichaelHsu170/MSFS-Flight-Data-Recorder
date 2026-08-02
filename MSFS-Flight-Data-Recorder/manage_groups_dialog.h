#pragma once

#include <QDialog>

class QListWidget;
class QListWidgetItem;

// Modal CRUD dialog for trip_groups: add/rename/delete groups. Each action
// commits immediately (no Ok/Cancel staging), same convention as the
// Delete Trip flow in TripHistoryPanel -- there is nothing to "cancel", so
// the dialog only has a single Close button.
class ManageGroupsDialog : public QDialog {
	Q_OBJECT
public:
	explicit ManageGroupsDialog(QWidget* parent = nullptr);

signals:
	// Emitted after a group is added, renamed, or deleted (i.e. as soon as the
	// change is committed to the database), not just when the dialog closes --
	// deletion in particular is irreversible, so callers showing group info
	// (e.g. TripHistoryPanel's table) should refresh immediately rather than
	// display a stale group on a trip until this dialog is closed.
	void groupsChanged();

private slots:
	void addGroup();
	void deleteSelectedGroup();
	void onItemChanged(QListWidgetItem* item);

private:
	// Rebuilds list_ from the database. clear() wipes every item (and with it
	// any selection), so without restoring one afterward, every reload() --
	// including the one onItemChanged() itself triggers right after a rename
	// commits -- left nothing selected, forcing the user to re-click the group
	// they were just working with before they could delete or rename it again.
	// selectGroupId, if >= 0, is the group to reselect after rebuilding
	// (addGroup() passes the newly created group's id); otherwise the
	// currently selected item's id (if any) is preserved automatically.
	void reload(int selectGroupId = -1);

	QListWidget* list_;
	// Guards onItemChanged against the setText() calls reload() makes to
	// populate the list, which would otherwise be misread as user renames.
	bool updating_ = false;
};
