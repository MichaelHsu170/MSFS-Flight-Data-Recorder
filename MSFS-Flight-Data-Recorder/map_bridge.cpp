#include "map_bridge.h"

MapBridge::MapBridge(QObject* parent) : QObject(parent) {}

void MapBridge::markerMoved(int index) {
	emit cursorIndexChanged(index);
}

void MapBridge::rangeChanged(int startIndex, int endIndex) {
	emit visibleRangeChanged(startIndex, endIndex);
}
