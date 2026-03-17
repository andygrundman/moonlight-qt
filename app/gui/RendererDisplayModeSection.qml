import QtQuick 2.9
import QtQuick.Controls 2.2
import QtQuick.Layouts 1.2

import StreamingPreferences 1.0
import SystemProperties 1.0

Column {
    id: root
    width: parent ? parent.width : 400
    spacing: 8

    property var languageChangedSignal

    visible: SystemProperties.hasDesktopEnvironment

    function reinitializeWindowMode() {
        if (!windowModeComboBox.visible) {
            return
        }

        windowModeComboBox.model = windowModeComboBox.createModel()
        windowModeComboBox.currentIndex = 0

        var savedWm = StreamingPreferences.windowMode
        for (var i = 0; i < windowModeComboBox.model.count; i++) {
            var thisWm = windowModeComboBox.model.get(i).val
            if (savedWm === thisWm) {
                windowModeComboBox.currentIndex = i
                break
            }
        }

        windowModeComboBox.activated(windowModeComboBox.currentIndex)
    }

    GridLayout {
        id: grid
        width: parent.width
        columns: 2
        columnSpacing: 10
        rowSpacing: 8

        Label {
            text: qsTr("Renderer")
            font.pointSize: 12
            wrapMode: Text.Wrap
            Layout.fillWidth: true
        }

        Label {
            text: qsTr("Renderer options")
            font.pointSize: 12
            wrapMode: Text.Wrap
            Layout.fillWidth: true
            visible: Qt.platform.os === "osx"
        }

        AutoResizingComboBox {
            id: rendererComboBox
            Layout.fillWidth: true
            textRole: "text"
            visible: Qt.platform.os === "osx"

            model: ListModel {
                id: rendererListModel

                ListElement {
                    text: qsTr("Metal (recommended)")
                    val: StreamingPreferences.RENDERER_VT_METAL
                }

                ListElement {
                    text: qsTr("AVSampleBuffer")
                    val: StreamingPreferences.RENDERER_AVSAMPLEBUFFER
                }
            }

            Component.onCompleted: {
                if (!visible) {
                    return
                }

                currentIndex = 0
                for (var i = 0; i < rendererListModel.count; i++) {
                    if (StreamingPreferences.renderer === rendererListModel.get(i).val) {
                        currentIndex = i
                        break
                    }
                }

                activated(currentIndex)
            }

            onActivated: {
                StreamingPreferences.renderer = rendererListModel.get(currentIndex).val
            }

            ToolTip.delay: 1000
            ToolTip.timeout: 5000
            ToolTip.visible: hovered
            ToolTip.text: qsTr("Choose the macOS renderer backend.")
        }

        AutoResizingComboBox {
            id: rendererOptionsComboBox
            Layout.fillWidth: true
            textRole: "text"
            visible: Qt.platform.os === "osx"
            enabled: !rendererComboBox.visible ||
                     StreamingPreferences.renderer === StreamingPreferences.RENDERER_VT_METAL

            model: ListModel {
                id: rendererOptionsListModel

                ListElement {
                    text: qsTr("Frames in flight: 3 (recommended)")
                    val: 3
                }

                ListElement {
                    text: qsTr("Frames in flight: 2")
                    val: 2
                }
            }

            Component.onCompleted: {
                if (!visible) {
                    return
                }

                currentIndex = 0
                for (var i = 0; i < rendererOptionsListModel.count; i++) {
                    if (StreamingPreferences.vtMetalFramesInFlight === rendererOptionsListModel.get(i).val) {
                        currentIndex = i
                        break
                    }
                }

                activated(currentIndex)
            }

            onActivated: {
                StreamingPreferences.vtMetalFramesInFlight = rendererOptionsListModel.get(currentIndex).val
            }

            ToolTip.delay: 1000
            ToolTip.timeout: 5000
            ToolTip.visible: hovered
            ToolTip.text: qsTr("Sets the maximum number of frames in flight for the Metal CPU to GPU pipeline. 2 frames has lower latency. 3 frames is often more stable at high frame rates.")
        }

        Label {
            text: qsTr("Display mode")
            font.pointSize: 12
            wrapMode: Text.Wrap
            Layout.fillWidth: true
        }

        Item {
            Layout.fillWidth: true
            Layout.preferredHeight: metalHudCheck.visible ? metalHudCheck.implicitHeight : 0

            CheckBox {
                id: metalHudCheck
                anchors.left: parent.left
                anchors.verticalCenter: parent.verticalCenter

                text: qsTr("Show Metal Performance HUD")
                font.pointSize: 12
                visible: Qt.platform.os === "osx" &&
                         (!rendererComboBox.visible ||
                          StreamingPreferences.renderer === StreamingPreferences.RENDERER_VT_METAL)
                checked: StreamingPreferences.showMetalPerformanceHud
                onCheckedChanged: {
                    StreamingPreferences.showMetalPerformanceHud = checked
                }

                ToolTip.delay: 1000
                ToolTip.timeout: 5000
                ToolTip.visible: hovered
                ToolTip.text: qsTr("Display Apple's Metal Performance HUD when viewing stats. Use Shift-F10 to cycle additional views.")
            }
        }

        AutoResizingComboBox {
            id: windowModeComboBox
            Layout.fillWidth: true
            enabled: !SystemProperties.rendererAlwaysFullScreen
            visible: SystemProperties.hasDesktopEnvironment
            hoverEnabled: true
            textRole: "text"

            function createModel() {
                var model = Qt.createQmlObject('import QtQuick 2.0; ListModel {}', root, '')

                if (Qt.platform.os !== "osx") {
                    model.append({
                        text: qsTr("Fullscreen"),
                        val: StreamingPreferences.WM_FULLSCREEN
                    })
                }

                model.append({
                    text: qsTr("Borderless windowed"),
                    val: StreamingPreferences.WM_FULLSCREEN_DESKTOP
                })

                model.append({
                    text: qsTr("Windowed"),
                    val: StreamingPreferences.WM_WINDOWED
                })

                for (var i = 0; i < model.count; i++) {
                    var thisWm = model.get(i).val
                    if (thisWm === StreamingPreferences.recommendedFullScreenMode) {
                        model.get(i).text += " " + qsTr("(Recommended)")
                        model.move(i, 0, 1)
                        break
                    }
                }

                return model
            }

            Component.onCompleted: {
                reinitializeWindowMode()

                if (languageChangedSignal) {
                    languageChangedSignal.connect(reinitializeWindowMode)
                }
            }

            onActivated: {
                StreamingPreferences.windowMode = model.get(currentIndex).val
            }

            ToolTip.delay: 1000
            ToolTip.timeout: 5000
            ToolTip.visible: hovered
            ToolTip.text: Qt.platform.os === "osx"
                ? qsTr("Borderless windowed generally provides the best performance. When used with an external VRR display, VRR will be enabled automatically.")
                : qsTr("Fullscreen generally provides the best performance, but borderless windowed may work better with features like Alt+Tab, screenshot tools, and overlays.")
        }

        Item {
            Layout.fillWidth: true
        }

        CheckBox {
            id: vsyncCheck
            Layout.columnSpan: 2
            Layout.fillWidth: true
            hoverEnabled: true
            text: qsTr("V-Sync")
            font.pointSize: 12
            checked: StreamingPreferences.enableVsync
            enabled: Qt.platform.os === "osx"
                ? (StreamingPreferences.windowMode === StreamingPreferences.WM_FULLSCREEN_DESKTOP)
                : true

            onCheckedChanged: {
                StreamingPreferences.enableVsync = checked
            }

            ToolTip.delay: 1000
            ToolTip.timeout: 5000
            ToolTip.visible: hovered
            ToolTip.text: Qt.platform.os === "osx"
                ? qsTr("V-Sync is always enabled on macOS in windowed mode and when using VRR. It can be disabled in borderless mode.")
                : qsTr("Disabling V-Sync allows sub-frame rendering latency, but it can display visible tearing")
        }

        CheckBox {
            id: framePacingCheck
            Layout.columnSpan: 2
            Layout.fillWidth: true
            hoverEnabled: true
            text: Qt.platform.os === "osx" ? qsTr("Frame pacing (always enabled)") : qsTr("Frame pacing")
            font.pointSize: 12
            enabled: Qt.platform.os === "osx" ? false : StreamingPreferences.enableVsync
            checked: Qt.platform.os === "osx" || (StreamingPreferences.enableVsync && StreamingPreferences.framePacing)

            onCheckedChanged: {
                StreamingPreferences.framePacing = checked
            }

            ToolTip.delay: 1000
            ToolTip.timeout: 5000
            ToolTip.visible: hovered
            ToolTip.text: Qt.platform.os === "osx"
                ? qsTr("Frame pacing is always enabled on macOS")
                : qsTr("Frame pacing reduces micro-stutter by delaying frames that come in too early")
        }
    }
}
