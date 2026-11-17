import QtQuick 2.15
import QtQuick.Controls 2.15
import QtQuick.Controls.Material 2.15
import QtQuick.Layouts 1.3

import Vedder.vesc.commands 1.0
import Vedder.vesc.utility 1.0

import "qrc:/mobile"

Item {
    id: root
    anchors.fill: parent
    anchors.margins: 10

    property Commands mCommands: VescIf.commands()

    readonly property color textColor: Utility.getAppHexColor("lightText")
    readonly property color cardColor: Qt.rgba(0.5, 0.5, 0.5, 0.16)

    // Same order as the states in boost_charge.lisp
    readonly property var stateNames: [
        "Ready to drive", "Plug in the charger", "Checking the battery", "Starting",
        "Ramping up", "Charging", "Topping off", "Charged", "Stopped by a fault",
        "Paused", "Bench test", "Charger connected"]
    readonly property var stateHints: [
        "Tap the button below, then plug in the charger.",
        "The motor is locked. Waiting for a steady charger voltage.",
        "",
        "Aligning the rotor. The wheel can move a little.",
        "",
        "",
        "Holding the target voltage while the current tapers off.",
        "You can unplug the charger.",
        "",
        "Tap Start to continue.",
        "Fixed current, no cell checks.",
        "Tap Start charging when you are ready."]
    readonly property var stateColors: [
        ["#607d8b", "#37474f"], ["#ffb300", "#f57c00"], ["#26c6da", "#00838f"], ["#26c6da", "#00897b"],
        ["#66bb6a", "#00897b"], ["#00c853", "#00a9a5"], ["#00bfa5", "#0288d1"], ["#2979ff", "#651fff"],
        ["#ef5350", "#b71c1c"], ["#78909c", "#455a64"], ["#ff8a50", "#e64a19"], ["#29b6f6", "#0277bd"]]

    // Same order as set-names in boost_charge.lisp
    readonly property var settingDefs: [
        {label: "Charge current", suffix: " A", dec: 1, from: 0.5, to: 200, step: 0.5},
        {label: "Charge each cell to", suffix: " V", dec: 2, from: 3.0, to: 4.35, step: 0.01},
        {label: "Stop when the current falls below", suffix: " A", dec: 2, from: 0, to: 50, step: 0.1},
        {label: "Lowest charger voltage (0 = automatic)", suffix: " V", dec: 1, from: 0, to: 400, step: 0.5},
        {label: "Highest charger voltage (0 = automatic)", suffix: " V", dec: 1, from: 0, to: 400, step: 0.5},
        {label: "Pack voltage limit (0 = automatic)", suffix: " V", dec: 1, from: 0, to: 400, step: 0.5},
        {label: "Switching frequency (0 = motor setting)", suffix: " kHz", dec: 1, from: 0, to: 40, step: 1},
        {label: "Charger power limit (0 = off)", suffix: " W", dec: 0, from: 0, to: 20000, step: 50},
        {label: "Battery current limit (0 = off)", suffix: " A", dec: 1, from: 0, to: 200, step: 0.5},
        {label: "Stop after (0 = off)", suffix: " Ah", dec: 1, from: 0, to: 1000, step: 1},
        {label: "Stop after (0 = off)", suffix: " h", dec: 1, from: 0, to: 48, step: 0.5},
        {label: "Coldest cell temperature", suffix: " °C", dec: 0, from: -20, to: 30, step: 1},
        {label: "Hottest cell temperature", suffix: " °C", dec: 0, from: 20, to: 70, step: 1},
        {label: "Motor temperature limit", suffix: " °C", dec: 0, from: 30, to: 150, step: 1},
        {label: "FET temperature limit", suffix: " °C", dec: 0, from: 30, to: 110, step: 1},
        {label: "Current ramp rate", suffix: " A/s", dec: 1, from: 0.1, to: 50, step: 0.5},
        {label: "Rotor alignment current", suffix: " A", dec: 1, from: 0, to: 20, step: 0.5},
        {label: "Rotor alignment time", suffix: " s", dec: 1, from: 0, to: 10, step: 0.5},
        {label: "Current loop Kp (0 = from motor)", suffix: " V/A", dec: 4, from: 0, to: 10, step: 0.001},
        {label: "Current loop Ki (0 = from motor)", suffix: "", dec: 2, from: 0, to: 5000, step: 1},
        {label: "Charger phase", suffix: "", dec: 0, from: 1, to: 3, step: 1},
        {label: "Over-current trip (0 = automatic)", suffix: " A", dec: 1, from: 0, to: 400, step: 1},
        {label: "Start automatically", suffix: "", dec: 0, from: 0, to: 1, step: 1},
        {label: "Charger voltage correction while charging (0 = board default)", suffix: " x", dec: 4, from: 0, to: 1.2, step: 0.001},
        {label: "Battery voltage correction while charging (0 = board default)", suffix: " x", dec: 4, from: 0, to: 1.2, step: 0.001}
    ]
    readonly property int idxPhase: 20
    readonly property int idxAuto: 22
    readonly property var basicGroup: [
        {title: "Basics", color: "#00c853", basic: true, items: [0, 1]}]
    readonly property var advancedGroups: [
        {title: "Charger", color: "#ffb300", basic: false, items: [3, 4, 7]},
        {title: "Limits", color: "#ef5350", basic: false, items: [2, 5, 8, 9, 10, 11, 12, 13, 14]},
        {title: "Converter", color: "#29b6f6", basic: false, items: [6, 15, 16, 17, 18, 19, 21, 23, 24]}]

    property bool settingsLoaded: false
    property bool loading: false
    property bool dirty: false
    property bool showAdvanced: false
    property var settingValues: []
    property int settingsRev: 0

    property int chgState: 0
    property bool locked: false
    property int trip: 0
    property real vSup: 0
    property real iIn: 0
    property real vBat: 0
    property real iSet: 0
    property real cellMin: 0
    property real cellMax: 0
    property real tCell: 0
    property real bmsAge: 999
    property real tMot: 0
    property real tFet: 0
    property real ah: 0
    property real wh: 0
    property real secs: 0
    property real duty: 0
    property var vPhase: [0, 0, 0]
    property int cells: 0
    property bool useBms: true
    property string message: ""
    property bool online: false
    property int flowPos: 0

    readonly property bool charging: (chgState >= 3 && chgState <= 6) || chgState == 10
    readonly property bool bmsOk: bmsAge <= 2
    readonly property real cellTarget: settingsLoaded ? settingValues[1] : 4.2
    readonly property real packTarget: cells * cellTarget
    // The bar runs from 3.0 V per cell to the target
    readonly property real fillLevel: cells > 0 && cellTarget > 3.0 ?
        Math.max(0, Math.min(1, (vBat / cells - 3.0) / (cellTarget - 3.0))) : 0
    readonly property int step: !online || chgState === 0 ? 0 : (chgState === 1 ? 1 : (chgState === 7 ? 3 : 2))

    function sendCode(str) {
        // Null-terminated so that the script can read it as a string
        mCommands.sendCustomAppData(str + "\0")
    }

    function setSetting(i, v) {
        var vals = settingValues.slice()
        vals[i] = v
        settingValues = vals
        dirty = true
        sendCode("(ui-set " + i + " " + v.toFixed(4) + ")")
    }

    function fmtTime(s) {
        var h = Math.floor(s / 3600)
        var m = Math.floor((s % 3600) / 60)
        return h + ":" + (m < 10 ? "0" : "") + m
    }

    // Rounded first, so that a small negative reading is not shown as -0
    function fmtPower() {
        return String(Math.round(vSup * iIn)) + " W"
    }

    function fmtTemp(t) {
        return t < -40 ? "–" : t.toFixed(0) + "°"
    }

    readonly property var tileLabels: ["Charger", "Battery", "Current", "Power", "Cells", "Temperature", "Charged", "Time"]
    readonly property var tileColors: ["#ffb300", "#00c853", "#29b6f6", "#ab47bc", "#00bfa5", "#ff7043", "#9ccc65", "#7986cb"]

    function tileValue(i) {
        switch (i) {
        // The charger side is only measured in charge mode
        case 0: return chgState === 0 ? "–" : vSup.toFixed(1) + " V"
        case 1: return vBat.toFixed(1) + " V"
        case 2: return chgState === 0 ? "–" : iIn.toFixed(1) + " A"
        case 3: return chgState === 0 ? "–" : fmtPower()
        case 4: return bmsOk ? cellMin.toFixed(2) + " – " + cellMax.toFixed(2) + " V" :
                               (cells > 0 ? (vBat / cells).toFixed(2) + " V" : "–")
        case 5: return fmtTemp(tFet) + " / " + fmtTemp(tMot)
        case 6: return ah.toFixed(2) + " Ah"
        case 7: return fmtTime(secs)
        }
        return ""
    }

    function tileNote(i) {
        switch (i) {
        case 0: return "on phase " + (settingsLoaded ? settingValues[idxPhase].toFixed(0) : "?")
        case 1: return cells + " cells, full at " + packTarget.toFixed(1) + " V"
        case 2: return "asking for " + iSet.toFixed(1) + " A"
        case 3: return "from the charger"
        case 4: return bmsOk ? "lowest and highest cell" : "average, no BMS data"
        case 5: return "FET / motor" + (bmsOk ? ", cells " + fmtTemp(tCell) : "")
        case 6: return wh.toFixed(0) + " Wh"
        case 7: return "hours : minutes"
        }
        return ""
    }

    Component.onCompleted: sendCode("(send-settings)")

    // Keep asking until the script answers
    Timer {
        interval: 1000
        running: !settingsLoaded
        repeat: true
        onTriggered: sendCode("(send-settings)")
    }

    // The script only answers, see ui-poll in boost_charge.lisp
    Timer {
        interval: 250
        running: true
        repeat: true
        onTriggered: sendCode("(ui-poll)")
    }

    // No status for 2 s means the script is not running
    Timer {
        id: onlineTimer
        interval: 2000
        onTriggered: online = false
    }

    Timer {
        interval: 180
        running: charging
        repeat: true
        onTriggered: flowPos = (flowPos + 1) % 5
    }

    Connections {
        target: mCommands

        function onCustomAppDataReceived(data) {
            var str = data.toString()
            var t = str.split(" ")

            if (t[0] === "st") {
                chgState = Number(t[1])
                locked = Number(t[2]) === 1
                trip = Number(t[3])
                vSup = Number(t[4])
                iIn = Number(t[5])
                vBat = Number(t[6])
                iSet = Number(t[7])
                cellMin = Number(t[8])
                cellMax = Number(t[9])
                tCell = Number(t[10])
                bmsAge = Number(t[11])
                tMot = Number(t[12])
                tFet = Number(t[13])
                ah = Number(t[14])
                wh = Number(t[15])
                secs = Number(t[16])
                duty = Number(t[17])
                vPhase = [Number(t[18]), Number(t[19]), Number(t[20])]
                cells = Number(t[21])
                useBms = Number(t[22]) === 1
                message = t.slice(23).join(" ").split(String.fromCharCode(0)).join("").trim()
                online = true
                onlineTimer.restart()
            } else if (t[0] === "set") {
                var vals = []
                for (var i = 0; i < settingDefs.length; i++) {
                    vals.push(Number(t[i + 1]))
                }
                loading = true
                settingValues = vals
                settingsRev++
                loading = false
                settingsLoaded = true
                dirty = false
            } else if (t[0] === "msg") {
                VescIf.emitStatusMessage(str.substring(4), true)
            }
        }
    }

    // One card of number settings. The first group also holds the phase
    // buttons and the auto start switch.
    Component {
        id: settingGroup

        Rectangle {
            property var grp: modelData
            Layout.fillWidth: true
            visible: grp.basic || showAdvanced
            implicitHeight: groupCol.implicitHeight + 24
            radius: 10
            color: cardColor

            ColumnLayout {
                id: groupCol
                anchors.fill: parent
                anchors.margins: 12
                spacing: 8

                Text {
                    text: grp.title
                    color: grp.color
                    font.bold: true
                    font.pointSize: 12
                }

                Repeater {
                    model: grp.items

                    delegate: ColumnLayout {
                        property int sIdx: modelData
                        Layout.fillWidth: true
                        spacing: 0

                        Text {
                            Layout.fillWidth: true
                            color: textColor
                            wrapMode: Text.WordWrap
                            text: settingDefs[sIdx].label
                        }

                        DoubleSpinBox {
                            id: spin
                            Layout.fillWidth: true
                            decimals: settingDefs[sIdx].dec
                            realFrom: settingDefs[sIdx].from
                            realTo: settingDefs[sIdx].to
                            realStepSize: settingDefs[sIdx].step
                            suffix: settingDefs[sIdx].suffix

                            // Assigned, not bound: editing would break a binding
                            Connections {
                                target: root
                                function onSettingsRevChanged() {
                                    spin.realValue = settingValues[sIdx]
                                }
                            }

                            onRealValueChanged: {
                                if (!loading && settingsLoaded && Math.abs(realValue - settingValues[sIdx]) > 1e-6) {
                                    setSetting(sIdx, realValue)
                                }
                            }
                        }
                    }
                }

                Text {
                    Layout.fillWidth: true
                    visible: grp.basic
                    color: textColor
                    wrapMode: Text.WordWrap
                    text: "Charger + is wired to motor phase"
                }

                RowLayout {
                    Layout.fillWidth: true
                    visible: grp.basic
                    spacing: 8

                    Repeater {
                        model: 3

                        delegate: Button {
                            Layout.fillWidth: true
                            Layout.preferredWidth: 10
                            readonly property bool picked: settingsLoaded && Math.round(settingValues[idxPhase]) === index + 1
                            Material.background: picked ? "#00c853" : "#546e7a"
                            Material.foreground: "white"
                            font.bold: true
                            text: (index + 1).toString()
                            onClicked: setSetting(idxPhase, index + 1)
                        }
                    }
                }

                RowLayout {
                    Layout.fillWidth: true
                    visible: grp.basic

                    Text {
                        Layout.fillWidth: true
                        color: textColor
                        wrapMode: Text.WordWrap
                        text: "Start charging as soon as the charger is plugged in"
                    }

                    Switch {
                        id: autoSwitch
                        onToggled: setSetting(idxAuto, checked ? 1 : 0)

                        Connections {
                            target: root
                            function onSettingsRevChanged() {
                                autoSwitch.checked = settingValues[idxAuto] > 0.5
                            }
                        }
                    }
                }
            }
        }
    }

    ColumnLayout {
        anchors.fill: parent
        spacing: 8

        TabBar {
            id: tabs
            Layout.fillWidth: true

            TabButton { text: "Charge" }
            TabButton { text: dirty ? "Setup •" : "Setup" }
        }

        StackLayout {
            Layout.fillWidth: true
            Layout.fillHeight: true
            currentIndex: tabs.currentIndex

            // ================= Charge =================
            ScrollView {
                id: chargeScroll
                contentWidth: availableWidth
                clip: true

                ColumnLayout {
                    width: chargeScroll.availableWidth
                    spacing: 10

                    // ---- What is going on ----
                    Rectangle {
                        Layout.fillWidth: true
                        implicitHeight: heroCol.implicitHeight + 32
                        radius: 14
                        gradient: Gradient {
                            orientation: Gradient.Horizontal
                            GradientStop { position: 0; color: online ? stateColors[chgState][0] : "#616161" }
                            GradientStop { position: 1; color: online ? stateColors[chgState][1] : "#424242" }
                        }

                        ColumnLayout {
                            id: heroCol
                            anchors.fill: parent
                            anchors.margins: 16
                            spacing: 4

                            Text {
                                Layout.fillWidth: true
                                color: "white"
                                font.pointSize: 20
                                font.bold: true
                                wrapMode: Text.WordWrap
                                text: online ? stateNames[chgState] : "Not connected"
                            }

                            Text {
                                Layout.fillWidth: true
                                color: "white"
                                opacity: 0.92
                                font.pointSize: 11
                                wrapMode: Text.WordWrap
                                visible: text !== ""
                                text: {
                                    if (!online) return "The charge script is not answering. Reconnect, or install the package again."
                                    if (message !== "") return message.charAt(0).toUpperCase() + message.slice(1)
                                    if (chgState === 11) return vSup.toFixed(1) + " V found. " + stateHints[11]
                                    return stateHints[chgState]
                                }
                            }

                            // Charger -> battery
                            RowLayout {
                                Layout.fillWidth: true
                                Layout.topMargin: 10
                                visible: online && chgState !== 0
                                spacing: 8

                                ColumnLayout {
                                    spacing: 0
                                    Text { color: "white"; opacity: 0.8; font.pointSize: 9; text: "CHARGER" }
                                    Text { color: "white"; font.pointSize: 15; font.bold: true; text: vSup.toFixed(1) + " V" }
                                }

                                Item { Layout.fillWidth: true }

                                Repeater {
                                    model: 5
                                    Rectangle {
                                        width: 9
                                        height: 9
                                        radius: 5
                                        color: "white"
                                        opacity: charging ? (index === flowPos ? 1.0 : 0.3) : 0.2
                                    }
                                }

                                Item { Layout.fillWidth: true }

                                ColumnLayout {
                                    spacing: 0
                                    Text { Layout.alignment: Qt.AlignRight; color: "white"; opacity: 0.8; font.pointSize: 9; text: "BATTERY" }
                                    Text { Layout.alignment: Qt.AlignRight; color: "white"; font.pointSize: 15; font.bold: true; text: vBat.toFixed(1) + " V" }
                                }
                            }

                            Text {
                                Layout.fillWidth: true
                                visible: charging
                                horizontalAlignment: Text.AlignHCenter
                                color: "white"
                                font.pointSize: 26
                                font.bold: true
                                text: fmtPower()
                            }

                            // Battery level
                            Rectangle {
                                Layout.fillWidth: true
                                Layout.topMargin: 6
                                visible: online && cells > 0
                                height: 10
                                radius: 5
                                color: Qt.rgba(0, 0, 0, 0.3)

                                Rectangle {
                                    width: Math.max(parent.height, parent.width * fillLevel)
                                    height: parent.height
                                    radius: 5
                                    color: "white"

                                    Behavior on width { NumberAnimation { duration: 400 } }
                                }
                            }

                            Text {
                                Layout.fillWidth: true
                                visible: online && cells > 0
                                color: "white"
                                opacity: 0.85
                                font.pointSize: 9
                                text: "Full at " + packTarget.toFixed(1) + " V  (" + cells + " cells × " + cellTarget.toFixed(2) + " V)"
                            }
                        }
                    }

                    // ---- Steps ----
                    RowLayout {
                        Layout.fillWidth: true
                        spacing: 4

                        Repeater {
                            model: ["Enable", "Plug in", "Charge", "Done"]

                            delegate: ColumnLayout {
                                Layout.fillWidth: true
                                Layout.preferredWidth: 10
                                spacing: 3

                                Rectangle {
                                    Layout.fillWidth: true
                                    height: 5
                                    radius: 3
                                    color: index <= step ? (chgState === 8 && index === 2 ? "#ef5350" : "#00c853") : cardColor
                                }

                                Text {
                                    Layout.fillWidth: true
                                    horizontalAlignment: Text.AlignHCenter
                                    color: textColor
                                    opacity: index === step ? 1.0 : 0.55
                                    font.bold: index === step
                                    font.pointSize: 9
                                    text: (index + 1) + ". " + modelData
                                }
                            }
                        }
                    }

                    // ---- The one button ----
                    Button {
                        Layout.fillWidth: true
                        Material.foreground: "white"
                        font.bold: true
                        visible: online && chgState === 0
                        Material.background: "#00c853"
                        text: "Enable charge mode"
                        onClicked: sendCode("(ui-lock 1)")
                    }

                    Button {
                        Layout.fillWidth: true
                        Material.foreground: "white"
                        font.bold: true
                        visible: online && (chgState === 11 || chgState === 9 || chgState === 7 || chgState === 8)
                        Material.background: chgState === 8 ? "#fb8c00" : "#00c853"
                        text: chgState === 8 ? "Clear the fault and try again" : (chgState === 7 ? "Charge again" : "Start charging")
                        onClicked: sendCode("(ui-start)")
                    }

                    Button {
                        Layout.fillWidth: true
                        Material.foreground: "white"
                        font.bold: true
                        visible: online && (charging || chgState === 2)
                        Material.background: "#e53935"
                        text: "Stop"
                        onClicked: sendCode("(ui-stop)")
                    }

                    Button {
                        Layout.fillWidth: true
                        Material.foreground: "white"
                        font.bold: true
                        visible: online && locked && !charging && chgState !== 2
                        Material.background: "#546e7a"
                        text: "Back to motor mode (unplug the charger first)"
                        // With voltage still on the phase the firmware can not
                        // always tell a charger from leakage, so the user has to
                        onClicked: vSup > 5.0 ? unlockDialog.open() : sendCode("(ui-lock 0)")
                    }

                    // ---- Numbers ----
                    GridLayout {
                        Layout.fillWidth: true
                        columns: 2
                        columnSpacing: 8
                        rowSpacing: 8

                        Repeater {
                            model: tileLabels.length

                            delegate: Rectangle {
                                Layout.fillWidth: true
                                Layout.preferredWidth: 10
                                implicitHeight: tileCol.implicitHeight + 18
                                radius: 10
                                color: cardColor

                                Rectangle {
                                    width: 4
                                    height: parent.height - 16
                                    radius: 2
                                    x: 6
                                    y: 8
                                    color: tileColors[index]
                                }

                                ColumnLayout {
                                    id: tileCol
                                    anchors.fill: parent
                                    anchors.leftMargin: 18
                                    anchors.rightMargin: 8
                                    anchors.topMargin: 9
                                    anchors.bottomMargin: 9
                                    spacing: 0

                                    Text {
                                        color: tileColors[index]
                                        font.pointSize: 9
                                        font.bold: true
                                        text: tileLabels[index].toUpperCase()
                                    }

                                    Text {
                                        Layout.fillWidth: true
                                        color: textColor
                                        font.pointSize: 15
                                        font.bold: true
                                        elide: Text.ElideRight
                                        text: tileValue(index)
                                    }

                                    Text {
                                        Layout.fillWidth: true
                                        color: textColor
                                        opacity: 0.6
                                        font.pointSize: 9
                                        elide: Text.ElideRight
                                        text: tileNote(index)
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // ================= Setup =================
            ColumnLayout {
            spacing: 6

            Text {
                Layout.fillWidth: true
                visible: charging
                color: "#ffb300"
                wrapMode: Text.WordWrap
                text: "Stop charging to change the settings."
            }

            ScrollView {
                id: setupScroll
                Layout.fillWidth: true
                Layout.fillHeight: true
                contentWidth: availableWidth
                clip: true

                ColumnLayout {
                    width: setupScroll.availableWidth
                    spacing: 10
                    enabled: settingsLoaded && !charging

                    Repeater {
                        model: basicGroup
                        delegate: settingGroup
                    }

                    Button {
                        Layout.fillWidth: true
                        flat: true
                        text: showAdvanced ? "Hide advanced settings" : "Show advanced settings"
                        onClicked: showAdvanced = !showAdvanced
                    }

                    Repeater {
                        model: advancedGroups
                        delegate: settingGroup
                    }

                    // ---- Bench test ----
                    Rectangle {
                        Layout.fillWidth: true
                        visible: showAdvanced
                        implicitHeight: benchCol.implicitHeight + 24
                        radius: 10
                        color: cardColor

                        ColumnLayout {
                            id: benchCol
                            anchors.fill: parent
                            anchors.margins: 12
                            spacing: 8

                            Text {
                                text: "Bench test"
                                color: "#ff7043"
                                font.bold: true
                                font.pointSize: 12
                            }

                            Text {
                                Layout.fillWidth: true
                                color: textColor
                                wrapMode: Text.WordWrap
                                text: "Holds a fixed charger current without any cell checks. Set the pack voltage " +
                                      "limit under Limits first, and use a current limited lab supply."
                            }

                            DoubleSpinBox {
                                id: manualCurrent
                                Layout.fillWidth: true
                                decimals: 1
                                realFrom: 0.5
                                realTo: 200
                                realStepSize: 0.5
                                realValue: 1.0
                                suffix: " A"
                            }

                            Button {
                                Layout.fillWidth: true
                                Material.background: "#ff7043"
                                Material.foreground: "white"
                                font.bold: true
                                enabled: online && locked && !charging
                                text: "Start bench test"
                                onClicked: {
                                    sendCode("(ui-manual " + manualCurrent.realValue.toFixed(2) + ")")
                                    tabs.currentIndex = 0
                                }
                            }
                        }
                    }

                    // ---- Diagnostics ----
                    Rectangle {
                        Layout.fillWidth: true
                        visible: showAdvanced
                        implicitHeight: diagCol.implicitHeight + 24
                        radius: 10
                        color: cardColor

                        ColumnLayout {
                            id: diagCol
                            anchors.fill: parent
                            anchors.margins: 12
                            spacing: 8

                            Text {
                                text: "Diagnostics"
                                color: "#7986cb"
                                font.bold: true
                                font.pointSize: 12
                            }

                            Text {
                                Layout.fillWidth: true
                                color: textColor
                                wrapMode: Text.WordWrap
                                text: "Phase voltages: " + vPhase[0].toFixed(1) + " / " + vPhase[1].toFixed(1) + " / " +
                                      vPhase[2].toFixed(1) + " V\n" +
                                      "Boost duty: " + (duty * 100).toFixed(1) + " %\n" +
                                      "Firmware trip code: " + trip + "\n" +
                                      "BMS data: " + (bmsOk ? "live" : "none for " + bmsAge.toFixed(0) + " s")
                            }
                        }
                    }
                }
            }

            // Stays in view while there is something to save
            Text {
                Layout.fillWidth: true
                visible: dirty
                color: textColor
                opacity: 0.7
                font.pointSize: 9
                wrapMode: Text.WordWrap
                text: "Changes are in use now. Save them to keep them after a restart."
            }

            Button {
                Layout.fillWidth: true
                visible: dirty
                enabled: !charging
                Material.background: "#00c853"
                Material.foreground: "white"
                font.bold: true
                text: "Save changes"
                onClicked: sendCode("(ui-save)")
            }
            }
        }
    }

    Dialog {
        id: unlockDialog
        modal: true
        anchors.centerIn: parent
        width: parent.width - 20
        title: "Is the charger unplugged?"
        standardButtons: Dialog.Yes | Dialog.Cancel
        onAccepted: sendCode("(ui-lock 0)")

        Text {
            width: unlockDialog.availableWidth
            color: textColor
            wrapMode: Text.WordWrap
            text: "There is still " + vSup.toFixed(1) + " V on the charger phase. Driving the motor " +
                  "with a charger connected shorts the charger."
        }
    }
}
