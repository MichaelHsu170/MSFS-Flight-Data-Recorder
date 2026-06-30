import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtGraphs

// Stacked timeline charts for the Trajectory View. One row per chart group,
// all sharing sharedXAxis (set from C++ in elapsed seconds since the loaded
// trip's first sample) so they scroll/zoom together -- including being
// zoomed to match the map's current viewport via ChartsPanel::setVisibleRange.
// Series are filled from C++ by objectName via QQuickItem::findChild -- see
// ChartsPanel::setDataset.
Item {
    id: root

    // Set from C++ (ChartsPanel::setCursorIndex) in the same elapsed-seconds
    // units as sharedXAxis when the map marker is dragged, so every row
    // highlights the same point in time at once.
    property real cursorTime: -1

    // Set from C++ (ChartsPanel::setDataset/setVisibleRange) -- true when
    // sharedXAxis currently spans the whole trip, false when it's been
    // zoomed to match a partial map viewport.
    property bool isFullRangeVisible: true

    // GraphsView's QML defaultProperty is "seriesList", not a normal item
    // child list -- a Rectangle declared directly inside a GraphsView {}
    // block gets swallowed into that list instead of being parented as an
    // Item, leaving an implicit `parent` reference null. So CursorLine takes
    // its target GraphsView explicitly and is declared as a sibling, not a
    // child, of the GraphsView it overlays.
    component CursorLine: Rectangle {
        property Item graphsView
        visible: graphsView !== null && root.cursorTime >= 0 && sharedXAxis.max > sharedXAxis.min
        x: graphsView ? graphsView.plotArea.x + (root.cursorTime - sharedXAxis.min) / (sharedXAxis.max - sharedXAxis.min) * graphsView.plotArea.width - width / 2 : 0
        y: graphsView ? graphsView.plotArea.y : 0
        width: 3
        height: graphsView ? graphsView.plotArea.height : 0
        color: "red"
        z: 10
    }

    // Solid bar on the right edge of a chart's plot area, marking the end of
    // the trip's data -- shown only when the chart's full time span is
    // visible (sharedXAxis covers the whole trip, not a map-zoomed subset),
    // so a solid edge always means "this is genuinely the end" rather than
    // just where the current zoom happens to stop.
    component EndOfTrajectoryLine: Rectangle {
        property Item graphsView
        visible: graphsView !== null && root.isFullRangeVisible
        x: graphsView ? graphsView.plotArea.x + graphsView.plotArea.width - width : 0
        y: graphsView ? graphsView.plotArea.y : 0
        width: 2
        height: graphsView ? graphsView.plotArea.height : 0
        color: "#555555"
        z: 9
    }

    // Qt Graphs' GraphsView has no built-in legend element in this Qt
    // version -- each series only exposes legendData (color/label) for a
    // custom legend to bind to, so build a simple colored-swatch row from
    // each chart's own series list instead.
    component ChartLegend: Row {
        id: legendRoot
        property var entries: []
        spacing: 14
        Repeater {
            model: legendRoot.entries
            delegate: Row {
                spacing: 4
                Rectangle { width: 10; height: 10; radius: 2; color: modelData.color; anchors.verticalCenter: parent.verticalCenter }
                Text { text: modelData.name; font.pixelSize: 11; color: "#333333"; anchors.verticalCenter: parent.verticalCenter }
            }
        }
    }

    // The legend used to sit in its own row stacked above the chart, but each
    // chart's own X-axis date/time labels render below its plot area and
    // bleed into the gap before the next ChartBlock -- with the legend row
    // sitting right there too, the two overlapped. Floating the legend as an
    // overlay anchored to the chart's own top-right corner keeps it clear of
    // every chart's axis labels regardless of how tall those labels get.
    component ChartBlock: Item {
        id: block
        property alias legendEntries: legendRow.entries
        default property alias content: inner.children

        Item {
            id: inner
            anchors.fill: parent
        }

        Rectangle {
            anchors.top: parent.top
            anchors.right: parent.right
            anchors.topMargin: 2
            anchors.rightMargin: 6
            width: legendRow.implicitWidth + 12
            height: legendRow.implicitHeight + 6
            radius: 4
            color: "#ccffffff"
            z: 20
            visible: legendRow.entries.length > 0

            ChartLegend {
                id: legendRow
                anchors.centerIn: parent
            }
        }
    }

    DateTimeAxis {
        id: sharedXAxis
        objectName: "sharedXAxis"
        // Must never be a zero-width range -- Qt Graphs' axis layout divides
        // by (max - min) on its very first paint, which happens before
        // ChartsPanel::setDataset() ever runs, so two equal placeholder
        // Date()s here crashes on startup with a divide-by-zero.
        min: new Date(2020, 0, 1, 0, 0, 0)
        max: new Date(2020, 0, 1, 0, 0, 1)
        labelFormat: "yyyy-MM-dd\nHH:mm:ss.zzz"
        titleVisible: false
    }

    // Shared by every chart row below so the gridlines stay thin/readable
    // instead of the (much heavier) Qt Graphs default -- the curves were
    // hard to read against the default grid weight.
    GraphsTheme {
        id: chartTheme
        grid.mainWidth: 1
        grid.subWidth: 1
        grid.mainColor: "#d9d9d9"
        grid.subColor: "#ececec"
    }

    // Charts must never scroll horizontally; only vertically, and only once
    // the stacked rows no longer fit the available height.
    ScrollView {
        anchors.fill: parent
        clip: true
        ScrollBar.horizontal.policy: ScrollBar.AlwaysOff
        ScrollBar.vertical.policy: ScrollBar.AsNeeded
        contentWidth: availableWidth

        ColumnLayout {
            width: parent ? parent.width : 0
            spacing: 2

            ChartBlock {
                Layout.fillWidth: true
                Layout.preferredHeight: 180
                Layout.minimumHeight: 160
                legendEntries: [
                    { color: "#1f77b4", name: "N1 #1" },
                    { color: "#ff7f0e", name: "N1 #2" },
                    { color: "#2ca02c", name: "N2 #1" },
                    { color: "#d62728", name: "N2 #2" }
                ]
                GraphsView {
                    id: chart1
                    anchors.fill: parent
                    axisX: sharedXAxis
                    theme: chartTheme
                    axisY: ValueAxis { min: 0; max: 110; labelFormat: "%.0f"; titleText: "N1/N2 (%)"; titleVisible: true }
                    LineSeries { objectName: "n1_1Series"; name: "N1 #1"; color: "#1f77b4" }
                    LineSeries { objectName: "n1_2Series"; name: "N1 #2"; color: "#ff7f0e" }
                    LineSeries { objectName: "n2_1Series"; name: "N2 #1"; color: "#2ca02c" }
                    LineSeries { objectName: "n2_2Series"; name: "N2 #2"; color: "#d62728" }
                }
                CursorLine { graphsView: chart1 }
                EndOfTrajectoryLine { graphsView: chart1 }
            }

            ChartBlock {
                Layout.fillWidth: true
                Layout.preferredHeight: 180
                Layout.minimumHeight: 160
                legendEntries: [
                    { color: "#9467bd", name: "Vertical Speed" }
                ]
                GraphsView {
                    id: chart2
                    anchors.fill: parent
                    axisX: sharedXAxis
                    theme: chartTheme
                    axisY: ValueAxis { min: -6000; max: 6000; labelFormat: "%.0f"; titleText: "V/S (ft/min)"; titleVisible: true }
                    LineSeries { objectName: "verticalSpeedSeries"; name: "Vertical Speed"; color: "#9467bd" }
                }
                CursorLine { graphsView: chart2 }
                EndOfTrajectoryLine { graphsView: chart2 }
            }

            ChartBlock {
                Layout.fillWidth: true
                Layout.preferredHeight: 180
                Layout.minimumHeight: 160
                legendEntries: [
                    { color: "#1f77b4", name: "Airspeed" },
                    { color: "#ff7f0e", name: "Ground Speed" }
                ]
                GraphsView {
                    id: chart3
                    anchors.fill: parent
                    axisX: sharedXAxis
                    theme: chartTheme
                    axisY: ValueAxis { min: 0; max: 400; labelFormat: "%.0f"; titleText: "Speed (kt)"; titleVisible: true }
                    LineSeries { objectName: "airspeedSeries"; name: "Airspeed"; color: "#1f77b4" }
                    LineSeries { objectName: "groundSpeedSeries"; name: "Ground Speed"; color: "#ff7f0e" }
                }
                CursorLine { graphsView: chart3 }
                EndOfTrajectoryLine { graphsView: chart3 }
            }

            ChartBlock {
                Layout.fillWidth: true
                Layout.preferredHeight: 180
                Layout.minimumHeight: 160
                legendEntries: [
                    { color: "#2ca02c", name: "Altitude" }
                ]
                GraphsView {
                    id: chart4
                    anchors.fill: parent
                    axisX: sharedXAxis
                    theme: chartTheme
                    axisY: ValueAxis { min: 0; max: 45000; labelFormat: "%.0f"; titleText: "Altitude (ft)"; titleVisible: true }
                    LineSeries { objectName: "altitudeSeries"; name: "Altitude"; color: "#2ca02c" }
                }
                CursorLine { graphsView: chart4 }
                EndOfTrajectoryLine { graphsView: chart4 }
            }

            ChartBlock {
                Layout.fillWidth: true
                Layout.preferredHeight: 180
                Layout.minimumHeight: 160
                legendEntries: [
                    { color: "#1f77b4", name: "Gear Handle" },
                    { color: "#ff7f0e", name: "Gear Pos 0" },
                    { color: "#2ca02c", name: "Gear Pos 1" },
                    { color: "#d62728", name: "Gear Pos 2" },
                    { color: "#9467bd", name: "On Ground 0" },
                    { color: "#8c564b", name: "On Ground 1" },
                    { color: "#e377c2", name: "On Ground 2" }
                ]
                GraphsView {
                    id: chart5
                    anchors.fill: parent
                    axisX: sharedXAxis
                    theme: chartTheme
                    axisY: ValueAxis { min: 0; max: 1.2; labelFormat: "%.1f"; titleText: "Gear"; titleVisible: true }
                    LineSeries { objectName: "gearHandleSeries"; name: "Gear Handle"; color: "#1f77b4" }
                    LineSeries { objectName: "gearPosition0Series"; name: "Gear Pos 0"; color: "#ff7f0e" }
                    LineSeries { objectName: "gearPosition1Series"; name: "Gear Pos 1"; color: "#2ca02c" }
                    LineSeries { objectName: "gearPosition2Series"; name: "Gear Pos 2"; color: "#d62728" }
                    LineSeries { objectName: "gearOnGround0Series"; name: "Gear On Ground 0"; color: "#9467bd" }
                    LineSeries { objectName: "gearOnGround1Series"; name: "Gear On Ground 1"; color: "#8c564b" }
                    LineSeries { objectName: "gearOnGround2Series"; name: "Gear On Ground 2"; color: "#e377c2" }
                }
                CursorLine { graphsView: chart5 }
                EndOfTrajectoryLine { graphsView: chart5 }
            }

            ChartBlock {
                Layout.fillWidth: true
                Layout.preferredHeight: 180
                Layout.minimumHeight: 160
                legendEntries: [
                    { color: "#1f77b4", name: "Brake" },
                    { color: "#ff7f0e", name: "Flaps Handle" },
                    { color: "#2ca02c", name: "Spoilers" }
                ]
                GraphsView {
                    id: chart6
                    anchors.fill: parent
                    axisX: sharedXAxis
                    theme: chartTheme
                    axisY: ValueAxis { min: 0; max: 8; labelFormat: "%.1f"; titleText: "Brake/Flaps/Spoilers"; titleVisible: true }
                    LineSeries { objectName: "brakeSeries"; name: "Brake"; color: "#1f77b4" }
                    LineSeries { objectName: "flapsSeries"; name: "Flaps Handle"; color: "#ff7f0e" }
                    LineSeries { objectName: "spoilersSeries"; name: "Spoilers"; color: "#2ca02c" }
                }
                CursorLine { graphsView: chart6 }
                EndOfTrajectoryLine { graphsView: chart6 }
            }

            ChartBlock {
                Layout.fillWidth: true
                Layout.preferredHeight: 180
                Layout.minimumHeight: 160
                legendEntries: [
                    { color: "#9467bd", name: "Fuel Weight" }
                ]
                GraphsView {
                    id: chart7
                    anchors.fill: parent
                    axisX: sharedXAxis
                    theme: chartTheme
                    axisY: ValueAxis { min: 0; max: 30000; labelFormat: "%.0f"; titleText: "Fuel Weight (lb)"; titleVisible: true }
                    LineSeries { objectName: "fuelWeightSeries"; name: "Fuel Weight"; color: "#9467bd" }
                }
                CursorLine { graphsView: chart7 }
                EndOfTrajectoryLine { graphsView: chart7 }
            }
        }
    }
}
