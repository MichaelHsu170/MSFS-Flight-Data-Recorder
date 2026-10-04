import QtQuick
import QtQuick.Controls.Basic
import QtQuick.Layouts
import QtGraphs

Item {
    id: root

    property real cursorTime: -1
    property bool isFullRangeVisible: true

    // tooltipModel  -- JS object returned by chartsBridge.valueAt(), null when hidden
    // tooltipSeries -- series descriptors for the chart currently under the cursor
    // tooltipPos    -- root-coordinate position of the mouse
    property var   tooltipModel:  null
    property var   tooltipSeries: null
    property point tooltipPos:    Qt.point(0, 0)

    // Format one series value from tooltipModel using its descriptor.
    //   s = { key, unit, decimals, signed, isBool }
    function formatValue(s, m) {
        if (!m) return ""
        var v = m[s.key]
        if (v === undefined || v === null) return ""
        if (s.isBool) return v ? "Y" : "N"
        var prefix = (s.signed && v >= 0) ? "+" : ""
        var suffix = s.unit ? " " + s.unit : ""
        return prefix + v.toFixed(s.decimals) + suffix
    }

    // Set by ChartsPanel::setEngine() (chartEngineSpec() in chart_data.h):
    // { count, speed/load Label, Unit, Decimals, AxisTitle }; undefined with no
    // data loaded.
    property var engineSpec: undefined
    // With no data, every chart hides its axes and grid and says why.
    readonly property bool hasData: engineSpec !== undefined
    // Shown over every chart with no data; set by ChartsPanel::setDataset():
    // "Loading…" while a trip loads, "No data recorded" for a trip with no
    // point (a live trip just started, or one that recorded none).
    property string noDataText: "No trip selected"

    // The engine power chart's series descriptors, in its LineSeries order
    // (engine 1..4 speed, then engine 1..4 load); engines past
    // engineSpec.count aren't shown.
    function engineSeries() {
        var spec = root.engineSpec
        var count = spec ? spec.count : 0
        var keys = {
            speed: ["engSpeed1", "engSpeed2", "engSpeed3", "engSpeed4"],
            load:  ["engLoad1", "engLoad2", "engLoad3", "engLoad4"]
        }
        var colors = {
            speed: ["#1f77b4", "#ff7f0e", "#9467bd", "#8c564b"],
            load:  ["#2ca02c", "#d62728", "#e377c2", "#17becf"]
        }
        var list = []
        var quantities = ["speed", "load"]
        for (var q = 0; q < quantities.length; q++) {
            var name = quantities[q]
            for (var i = 0; i < 4; i++) {
                list.push({
                    key: keys[name][i],
                    label: count > 0 ? spec[name + "Label"] + " #" + (i + 1) : "",
                    color: colors[name][i],
                    unit: count > 0 ? spec[name + "Unit"] : "",
                    decimals: count > 0 ? spec[name + "Decimals"] : 0,
                    signed: false, isBool: false,
                    shown: i < count
                })
            }
        }
        return list
    }

    // The engine power chart's right-hand axis, for its load series.
    ValueAxis {
        id: engLoadAxis
        objectName: "engLoadYAxis"
        alignment: Qt.AlignRight
        visible: root.engineSpec !== undefined && root.engineSpec.count > 0
        // Hidden with no data like every chart's axes (see ChartBlock).
        lineVisible: root.hasData
        gridVisible: root.hasData
        subGridVisible: root.hasData
        labelFormat: "%.0f"
        titleText: visible ? root.engineSpec.loadAxisTitle : ""
        titleVisible: root.hasData
    }

    DateTimeAxis {
        id: driverXAxis
        objectName: "sharedXAxis"
        min: new Date(2020, 0, 1, 0, 0, 0)
        max: new Date(2020, 0, 1, 0, 0, 1)
        labelFormat: "yyyy-MM-dd\nHH:mm:ss.zzz"
        titleVisible: false
    }

    component SyncedXAxis: DateTimeAxis {
        labelFormat: "yyyy-MM-dd\nHH:mm:ss.zzz"
        titleVisible: false
    }

    component CursorLine: Rectangle {
        property Item graphsView
        visible: graphsView !== null && root.cursorTime >= 0 && driverXAxis.max > driverXAxis.min
        x: graphsView ? graphsView.plotArea.x + (root.cursorTime - driverXAxis.min) / (driverXAxis.max - driverXAxis.min) * graphsView.plotArea.width - width / 2 : 0
        y: graphsView ? graphsView.plotArea.y : 0
        width: 3
        height: graphsView ? graphsView.plotArea.height : 0
        color: "red"
        z: 10
    }

    component EndOfTrajectoryLine: Rectangle {
        property Item graphsView
        objectName: "endOfTrajectoryLine"
        visible: graphsView !== null && root.hasData && root.isFullRangeVisible
        x: graphsView ? graphsView.plotArea.x + graphsView.plotArea.width - width : 0
        y: graphsView ? graphsView.plotArea.y : 0
        width: 2
        height: graphsView ? graphsView.plotArea.height : 0
        color: "#555555"
        z: 9
    }

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

    component ChartBlock: Item {
        id: block
        Layout.fillWidth: true
        Layout.preferredHeight: 180
        Layout.minimumHeight: 160
        // One entry per LineSeries declared in the block, in the same order:
        //   { key, label, legend, color, unit, decimals, signed, isBool, shown }
        // Colors and names the lines, fills the legend (legend, or label when
        // there's none), and drives the hover tooltip -- only these series are
        // shown when this chart is under the cursor. shown: false hides a line
        // (default shown).
        property var series: []
        readonly property var shownSeries: series.filter(function (s) { return s.shown !== false })
        // Centered over the plot when non-empty, e.g. when a trip has no data
        // for this chart. With no data at all, root.noDataText replaces it.
        property string emptyText: ""
        readonly property string shownEmptyText: root.hasData ? emptyText : root.noDataText
        property alias axisY: view.axisY
        default property alias lineSeries: view.seriesList

        function applySeries() {
            for (var i = 0; i < series.length && i < view.seriesList.length; i++) {
                view.seriesList[i].color = series[i].color
                view.seriesList[i].name = series[i].legend || series[i].label
                view.seriesList[i].visible = series[i].shown !== false
            }
        }
        onSeriesChanged: applySeries()
        Component.onCompleted: applySeries()

        MouseArea {
            anchors.fill: parent
            hoverEnabled: true
            acceptedButtons: Qt.NoButton
            z: 5

            onExited: { root.tooltipModel = null; root.tooltipSeries = null }

            onPositionChanged: function(mouse) {
                var pa = view.plotArea
                if (pa.width <= 0) return
                var minMs = driverXAxis.min.getTime()
                var maxMs = driverXAxis.max.getTime()
                if (maxMs <= minMs) return
                // Clamp to the plot area rather than hiding when the cursor strays
                // into the axis-label margins. pa.x is a float; strict inequality
                // against integer mouse.x would intermittently classify pixels that
                // are visually inside the plot as "outside" and hide the tooltip.
                var plotX = Math.max(pa.x, Math.min(pa.x + pa.width, mouse.x))
                var fraction = (plotX - pa.x) / pa.width
                var data = chartsBridge.valueAt(minMs + fraction * (maxMs - minMs))
                root.tooltipModel  = (data && data.timeStr !== undefined) ? data : null
                // Set tooltipSeries here too in case onPositionChanged fires before
                // onEntered when transitioning quickly between adjacent charts.
                root.tooltipSeries = root.tooltipModel ? block.shownSeries : null
                if (root.tooltipModel) {
                    var p = mapToItem(root, mouse.x, mouse.y)
                    root.tooltipPos = Qt.point(p.x, p.y)
                }
            }
        }

        // Space right of the plot. The engine power chart's right-hand axis
        // takes space even while hidden, so every other chart matches its
        // inset to keep the time axes lined up.
        readonly property real plotRightInset: view.width - view.plotArea.x - view.plotArea.width

        GraphsView {
            id: view
            anchors.fill: parent
            axisX: SyncedXAxis {}
            theme: chartTheme
            Binding on marginRight {
                when: block !== engineBlock
                value: engineBlock.plotRightInset
            }
        }
        // No data: no axis, title, labels, grid or legend -- they'd only
        // describe a made-up range.
        // (visible alone leaves the axis line drawn.)
        Binding { target: view.axisX; property: "visible"; value: root.hasData }
        Binding { target: view.axisX; property: "lineVisible"; value: root.hasData }
        Binding { target: view.axisX; property: "gridVisible"; value: root.hasData }
        Binding { target: view.axisX; property: "subGridVisible"; value: root.hasData }
        Binding { target: view.axisY; property: "visible"; value: root.hasData }
        Binding { target: view.axisY; property: "lineVisible"; value: root.hasData }
        Binding { target: view.axisY; property: "titleVisible"; value: root.hasData }
        Binding { target: view.axisY; property: "gridVisible"; value: root.hasData }
        Binding { target: view.axisY; property: "subGridVisible"; value: root.hasData }
        CursorLine { graphsView: view }
        EndOfTrajectoryLine { graphsView: view }

        Text {
            objectName: "emptyText"
            visible: block.shownEmptyText !== ""
            text: block.shownEmptyText
            x: view.plotArea.x + (view.plotArea.width - width) / 2
            y: view.plotArea.y + (view.plotArea.height - height) / 2
            font.pixelSize: 13
            color: "#777777"
            z: 10
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
            objectName: "legend"
            visible: root.hasData && legendRow.entries.length > 0
            ChartLegend {
                id: legendRow
                anchors.centerIn: parent
                entries: block.shownSeries.map(function (s) { return { color: s.color, name: s.legend || s.label } })
            }
        }
    }

    GraphsTheme {
        id: chartTheme
        grid.mainWidth: 1
        grid.subWidth: 1
        grid.mainColor: "#d9d9d9"
        grid.subColor: "#ececec"
    }

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
                id: engineBlock
                objectName: "engineBlock"
                series: root.engineSeries()
                emptyText: root.engineSpec !== undefined && root.engineSpec.count === 0 ? "No engine power data recorded" : ""
                axisY: ValueAxis {
                    objectName: "engSpeedYAxis"
                    labelFormat: "%.0f"
                    titleText: root.engineSpec !== undefined && root.engineSpec.count > 0 ? root.engineSpec.speedAxisTitle : "Engine Power"
                    titleVisible: true
                }
                LineSeries { objectName: "engSpeed1Series" }
                LineSeries { objectName: "engSpeed2Series" }
                LineSeries { objectName: "engSpeed3Series" }
                LineSeries { objectName: "engSpeed4Series" }
                LineSeries { objectName: "engLoad1Series"; axisY: engLoadAxis }
                LineSeries { objectName: "engLoad2Series"; axisY: engLoadAxis }
                LineSeries { objectName: "engLoad3Series"; axisY: engLoadAxis }
                LineSeries { objectName: "engLoad4Series"; axisY: engLoadAxis }
            }

            ChartBlock {
                series: [
                    { key: "vs", label: "V/S", legend: "Vertical Speed", color: "#9467bd", unit: "ft/min", decimals: 0, signed: true, isBool: false }
                ]
                axisY: ValueAxis { objectName: "vsYAxis"; labelFormat: "%.0f"; titleText: "V/S (ft/min)"; titleVisible: true }
                LineSeries { objectName: "verticalSpeedSeries" }
            }

            ChartBlock {
                series: [
                    { key: "ias", label: "Airspeed", color: "#1f77b4", unit: "kt", decimals: 0, signed: false, isBool: false },
                    { key: "gs", label: "Ground Speed", color: "#ff7f0e", unit: "kt", decimals: 0, signed: false, isBool: false }
                ]
                axisY: ValueAxis { objectName: "speedYAxis"; labelFormat: "%.0f"; titleText: "Speed (kt)"; titleVisible: true }
                LineSeries { objectName: "airspeedSeries" }
                LineSeries { objectName: "groundSpeedSeries" }
            }

            ChartBlock {
                series: [
                    { key: "alt", label: "Altitude", color: "#2ca02c", unit: "ft", decimals: 0, signed: false, isBool: false }
                ]
                axisY: ValueAxis { objectName: "altYAxis"; labelFormat: "%.0f"; titleText: "Altitude (ft)"; titleVisible: true }
                LineSeries { objectName: "altitudeSeries" }
            }

            ChartBlock {
                series: [
                    { key: "gearHandle", label: "Gear Handle", color: "#1f77b4", unit: "", decimals: 1, signed: false, isBool: false },
                    { key: "gearPos0", label: "Gear Pos 0", color: "#ff7f0e", unit: "", decimals: 1, signed: false, isBool: false },
                    { key: "gearPos1", label: "Gear Pos 1", color: "#2ca02c", unit: "", decimals: 1, signed: false, isBool: false },
                    { key: "gearPos2", label: "Gear Pos 2", color: "#d62728", unit: "", decimals: 1, signed: false, isBool: false },
                    { key: "onGnd0", label: "On Ground 0", color: "#9467bd", unit: "", decimals: 0, signed: false, isBool: true },
                    { key: "onGnd1", label: "On Ground 1", color: "#8c564b", unit: "", decimals: 0, signed: false, isBool: true },
                    { key: "onGnd2", label: "On Ground 2", color: "#e377c2", unit: "", decimals: 0, signed: false, isBool: true }
                ]
                axisY: ValueAxis { min: 0; max: 1.2; labelFormat: "%.1f"; titleText: "Gear"; titleVisible: true }
                LineSeries { objectName: "gearHandleSeries" }
                LineSeries { objectName: "gearPosition0Series" }
                LineSeries { objectName: "gearPosition1Series" }
                LineSeries { objectName: "gearPosition2Series" }
                LineSeries { objectName: "gearOnGround0Series" }
                LineSeries { objectName: "gearOnGround1Series" }
                LineSeries { objectName: "gearOnGround2Series" }
            }

            ChartBlock {
                series: [
                    { key: "brake", label: "Brake", color: "#1f77b4", unit: "", decimals: 0, signed: false, isBool: false },
                    { key: "flaps", label: "Flaps", legend: "Flaps Handle", color: "#ff7f0e", unit: "", decimals: 0, signed: false, isBool: false },
                    { key: "spoilers", label: "Spoilers", color: "#2ca02c", unit: "", decimals: 1, signed: false, isBool: false }
                ]
                axisY: ValueAxis { min: 0; max: 8; labelFormat: "%.1f"; titleText: "Brake/Flaps/Spoilers"; titleVisible: true }
                LineSeries { objectName: "brakeSeries" }
                LineSeries { objectName: "flapsSeries" }
                LineSeries { objectName: "spoilersSeries" }
            }

            ChartBlock {
                series: [
                    { key: "fuel", label: "Fuel Weight", color: "#9467bd", unit: "lb", decimals: 0, signed: false, isBool: false }
                ]
                axisY: ValueAxis { objectName: "fuelYAxis"; labelFormat: "%.0f"; titleText: "Fuel Weight (lb)"; titleVisible: true }
                LineSeries { objectName: "fuelWeightSeries" }
            }

            ChartBlock {
                series: [
                    { key: "pitch", label: "Pitch", color: "#e377c2", unit: "°", decimals: 1, signed: true, isBool: false }
                ]
                axisY: ValueAxis { objectName: "pitchYAxis"; labelFormat: "%.0f"; titleText: "Pitch (°)"; titleVisible: true }
                LineSeries { objectName: "pitchSeries" }
            }

            ChartBlock {
                series: [
                    { key: "bank", label: "Bank", color: "#17becf", unit: "°", decimals: 1, signed: true, isBool: false }
                ]
                axisY: ValueAxis { objectName: "bankYAxis"; labelFormat: "%.0f"; titleText: "Bank (°)"; titleVisible: true }
                LineSeries { objectName: "bankSeries" }
            }
        }
    }

    // Floating tooltip -- declared after ScrollView so it renders on top.
    // Shows only the series belonging to the chart currently under the cursor.
    Rectangle {
        visible: root.tooltipModel !== null && root.tooltipSeries !== null
        x: {
            var tx = root.tooltipPos.x + 14
            return (tx + width > root.width - 8) ? (root.tooltipPos.x - width - 14) : tx
        }
        y: Math.max(4, Math.min(root.tooltipPos.y - Math.round(height / 2), root.height - height - 4))
        z: 200
        width:  ttCol.implicitWidth  + 18
        height: ttCol.implicitHeight + 14
        color: "#e8161616"
        border.color: "#505050"
        border.width: 1
        radius: 5

        Column {
            id: ttCol
            anchors.centerIn: parent
            spacing: 4

            // Time header
            Text {
                text: root.tooltipModel ? root.tooltipModel.timeStr : ""
                color: "#ffcc44"
                font.pixelSize: 12
                font.bold: true
                font.family: "Consolas"
            }

            // One row per series in the hovered chart
            Repeater {
                model: root.tooltipSeries || []
                delegate: Row {
                    spacing: 6
                    Rectangle {
                        width: 8; height: 8; radius: 4
                        color: modelData.color
                        anchors.verticalCenter: parent.verticalCenter
                    }
                    Text {
                        text: modelData.label + "   " + root.formatValue(modelData, root.tooltipModel)
                        color: "#dddddd"
                        font.pixelSize: 11
                        font.family: "Consolas"
                    }
                }
            }
        }
    }
}
