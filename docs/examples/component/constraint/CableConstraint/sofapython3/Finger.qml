import QtQuick 2.4
import QtQuick.Controls 2.4
import QtQuick.Layouts 1.0
import SofaBasics 1.0
import SofaApplication 1.0
import QtCharts 2.3
import QtGraphicalEffects 1.0

SofaCanvas {
    id: root
    // The data we manipulate in the GUI:
    property var cableControllerValue: SofaApplication.sofaScene.get("/finger/cable/aCableActuator").getData("value")

    // Simple helper function for later...
    function setValue(val) {
        cableControllerValue.value = [val]
    }

    implicitWidth: 200
    implicitHeight: 100

    color: "transparent"
    ColumnLayout {
        anchors.fill: parent
        spacing: 10




//        /// Colored Sprite of the finger showing white, orange, red depending on cable displacement
//        Image {
//            id: stencil
//            source: "fingerStencil.png"
//            sourceSize: Qt.size(160,60)
//            ColorOverlay {
//                id: overlay
//                anchors.fill: stencil
//                source: stencil
//                color: "white"
//                function updateColor() {
//                    if (cableControllerValue.value[0] > 30)
//                        color = "red"
//                    else if (cableControllerValue.value[0] > 15)
//                        color = "orange"
//                    else
//                        color = "white"
//                }
//                Connections
//                {
//                    target: cableControllerValue
//                    onValueChanged: {
//                        overlay.updateColor() // updates the stencil color
//                    }
//                }
//            }
//        }





//        /// BUTTONS
//        RowLayout {
//            Layout.fillWidth: true
//            Button {
//                Layout.fillWidth: true
//                text: "unwind"
//                onClicked: {
//                    var val = Number(cableControllerValue.value[0])
//                    setValue(val - 1.0)
//                }
//            }
//            Button {
//                Layout.fillWidth: true
//                text: "wind"
//                onClicked: {
//                    var val = Number(cableControllerValue.value[0])
//                    setValue(val + 1.0)
//                }
//            }
//        }





        /// SLIDER
        Slider {
            Layout.fillWidth: true
            value: Number(cableControllerValue.value[0])
            from: 0
            to: 45
            onValueChanged: {
                if (pressed)
                    setValue(value)
            }
        }





       /// CHART
       ChartView {
           id: histogram
           title: "Cable displacement"
           implicitHeight: 200
           implicitWidth: 400
           legend.visible: false
           antialiasing: true
           animationOptions: ChartView.NoAnimation
           theme: ChartView.ChartThemeDark
           opacity: 0.7
           ValueAxis{
               id: axisX
               min: 0
               max: 100
               tickCount: 5
           }
           ValueAxis{
               id: axisY
               min: 0
               max: 45
           }

           SplineSeries {
               id: displacement
               name: "points"
               axisX: axisX
               axisY: axisY
               property var t: 0
               color: "red"
               Connections {
                   target: cableControllerValue
                   onValueChanged: {
                       // Add a point to the serie
                       displacement.append(displacement.t++, Number(cableControllerValue.value[0]))

                       // Remove first point if serie is full (moving window)
                       if (displacement.count > 100)
                           displacement.remove(0)

                       // update axis window
                       axisX.min = displacement.at(0).x
                       axisX.max = displacement.t
                   }
               }
           }
       }









    }
}
