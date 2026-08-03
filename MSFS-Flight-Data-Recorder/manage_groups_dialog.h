#pragma once

#include <QDialog>
#include <QListWidget>

class QDropEvent;

// Plain QListWidget with QAbstractItemView::InternalMove doesn't emit any
// reliable signal when the user finishes a drag-reorder: rowsMoved is never
// fired for it (Qt implements InternalMove as a remove+insert through
// QAbstractItemModel::dropMimeData, not moveRows/beginMoveRows/endMoveRows --
// a long-standing, documented Qt limitation), and rowsInserted/rowsRemoved
// fire mid-gesture in an order that isn't safe to rebuild the list from
// (Qt's own drop handling is still running further down the call stack).
// Overriding dropEvent() to let the base class finish the move and then
// emit our own signal is the standard, safe workaround.
class ReorderableListWidget : public QListWidget {
	Q_OBJECT
public:
	explicit ReorderableListWidget(QWidget* parent = nullptr);

signals:
	// Emitted after a completed internal drag-reorder, once the base class's
	// dropEvent() has fully applied it -- list_'s item order is final and
	// safe to read at this point.
	void reordered();

protected:
	void dropEvent(QDropEvent* event) override;
};

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
	// Fires once list_'s ReorderableListWidget::reordered() signal reports a
	// completed drag-reorder. Persists the list's new visual order as each
	// group's sort_order.
	void onListReordered();

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

	ReorderableListWidget* list_;
	// Guards onItemChanged/onListReordered against the setText()/addItem()
	// calls reload() makes to populate the list, which would otherwise be
	// misread as user edits.
	bool updating_ = false;
};
