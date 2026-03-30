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
            ToolTip.timeout: -1
            ToolTip.visible: hovered
            ToolTip.text:
                qsTr("Metal is the modern advanced renderer. All other options in this section require Metal.\n\n") +
                qsTr("AVSampleBuffer lets macOS control the rendering and can be used if Metal doesn't work for you.")
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
            ToolTip.timeout: -1
            ToolTip.visible: hovered
            ToolTip.text: qsTr("Use 2 frames for lower latency. 3 frames may be needed for higher framerates.")
        }

        Label {
            text: qsTr("Display mode")
            font.pointSize: 12
            wrapMode: Text.Wrap
            Layout.fillWidth: true
        }

        Label {
            text: qsTr("Frame pacing mode")
            font.pointSize: 12
            wrapMode: Text.Wrap
            Layout.fillWidth: true
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

                model.append({
                    text: qsTr("Fullscreen"),
                    val: StreamingPreferences.WM_FULLSCREEN
                })

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
            ToolTip.timeout: -1
            ToolTip.visible: hovered
            ToolTip.text: Qt.platform.os === "osx"
                ? qsTr("Borderless windowed generally provides the best performance. When used with an external VRR display, VRR will be enabled automatically.")
                : qsTr("Fullscreen generally provides the best performance, but borderless windowed may work better with features like Alt+Tab, screenshot tools, and overlays.")
        }

        AutoResizingComboBox {
            id: framePacingComboBox
            Layout.fillWidth: true
            hoverEnabled: true
            textRole: "text"
            enabled: !rendererComboBox.visible ||
                     StreamingPreferences.renderer === StreamingPreferences.RENDERER_VT_METAL

            model: ListModel {
                id: framePacingListModel

                ListElement {
                    text: qsTr("Immediate (recommended)")
                    val: StreamingPreferences.FRAME_PACING_IMMEDIATE
                }

                ListElement {
                    text: qsTr("Display-locked")
                    val: StreamingPreferences.FRAME_PACING_DISPLAY_LOCKED
                }
            }

            Component.onCompleted: {
                currentIndex = 0
                for (var i = 0; i < framePacingListModel.count; i++) {
                    if (StreamingPreferences.framePacingMode === framePacingListModel.get(i).val) {
                        currentIndex = i
                        break
                    }
                }

                activated(currentIndex)
            }

            onActivated: {
                StreamingPreferences.framePacingMode = framePacingListModel.get(currentIndex).val
            }

            ToolTip.delay: 1000
            ToolTip.timeout: -1
            ToolTip.visible: hovered
            ToolTip.text:
                qsTr("Immediate: renders frames as they arrive, either with or without vsync. When the stream fps drops below max, stutters may occur.\n\n") +
                qsTr("Display-locked: renders at the client refresh rate and displays new or repeat frames according to the PTS timestamp. Best for: lower fps content, video, film.")
        }

        Label {
            text: qsTr("Present mode")
            font.pointSize: 12
            wrapMode: Text.Wrap
            Layout.fillWidth: true
        }

        Label {
            text: qsTr("Stats")
            font.pointSize: 12
            wrapMode: Text.Wrap
            Layout.fillWidth: true
        }

        AutoResizingComboBox {
            id: presentModeComboBox
            Layout.fillWidth: true
            hoverEnabled: true
            textRole: "text"
            enabled: !rendererComboBox.visible ||
                     StreamingPreferences.renderer === StreamingPreferences.RENDERER_VT_METAL

            model: ListModel {
                id: presentModeListModel

                ListElement {
                    text: qsTr("Auto (recommended)")
                    val: StreamingPreferences.PRESENT_AUTO
                }

                ListElement {
                    text: qsTr("Fixed Vsync")
                    val: StreamingPreferences.PRESENT_FIXED
                }

                ListElement {
                    text: qsTr("VRR")
                    val: StreamingPreferences.PRESENT_VRR
                }

                ListElement {
                    text: qsTr("No Vsync")
                    val: StreamingPreferences.PRESENT_NO_VSYNC
                }
            }

            Component.onCompleted: {
                currentIndex = 0
                for (var i = 0; i < presentModeListModel.count; i++) {
                    if (StreamingPreferences.presentMode === presentModeListModel.get(i).val) {
                        currentIndex = i
                        break
                    }
                }

                activated(currentIndex)
            }

            onActivated: {
                StreamingPreferences.presentMode = presentModeListModel.get(currentIndex).val
            }

            ToolTip.delay: 1000
            ToolTip.timeout: -1
            ToolTip.visible: hovered
            ToolTip.text:
                qsTr("Auto: Will use VRR when possible, otherwise Fixed.\n\n") +
                qsTr("Fixed (Vsync): aligns frames to the vsync interval.\n\n") +
                qsTr("VRR: Frames are presented at varying intervals within the monitor's supported range. Low latency with no tearing.\n\n") +
                qsTr("No Vsync: presents frames with no delay, for the lowest latency at the cost of screen tearing.")
        }

        Column {
            spacing: 2

            CheckBox {
                id: performanceStatsCheck
                Layout.fillWidth: true
                hoverEnabled: true
                text: qsTr("Show performance stats")
                font.pointSize: 12
                checked: StreamingPreferences.showPerformanceOverlay

                onCheckedChanged: {
                    StreamingPreferences.showPerformanceOverlay = checked
                }

                ToolTip.delay: 1000
                ToolTip.timeout: -1
                ToolTip.visible: hovered
                ToolTip.text:
                    qsTr("Display real-time stream performance information while streaming.") + "\n\n" +
                    qsTr("You can toggle it at any time while streaming using Ctrl+Alt+Shift+S or Select+L1+R1+X.") + "\n\n" +
                    qsTr("The performance overlay is not supported on Steam Link or Raspberry Pi.")
            }

            CheckBox {
                id: performanceGraphsCheck
                Layout.fillWidth: true
                hoverEnabled: true
                text: qsTr("Show performance graphs")
                font.pointSize: 12
                checked: StreamingPreferences.showPerformanceGraphs
                enabled: !rendererComboBox.visible ||
                         StreamingPreferences.renderer === StreamingPreferences.RENDERER_VT_METAL

                onCheckedChanged: {
                    StreamingPreferences.showPerformanceGraphs = checked
                }

                ToolTip.delay: 1000
                ToolTip.timeout: -1
                ToolTip.visible: hovered
                ToolTip.text: qsTr(
                    "Displays realtime graphs for monitoring Frametime, Host Frametime, Present Delay, Dropped Frames, and Network Mbps. Click on a graph to view a larger version.")
            }

            CheckBox {
                id: developerUICheck
                Layout.fillWidth: true
                hoverEnabled: true
                text: qsTr("Enable Advanced UI")
                font.pointSize: 12
                checked: StreamingPreferences.enableDeveloperUI
                enabled: !rendererComboBox.visible ||
                         StreamingPreferences.renderer === StreamingPreferences.RENDERER_VT_METAL

                onCheckedChanged: {
                    StreamingPreferences.enableDeveloperUI = checked
                }

                ToolTip.delay: 1000
                ToolTip.timeout: -1
                ToolTip.visible: hovered
                ToolTip.text: qsTr(
                    "Change various rendering settings in realtime and view more detailed metrics. Use this to experiment with different frame pacing and present settings.")
            }
        }
    }
}
