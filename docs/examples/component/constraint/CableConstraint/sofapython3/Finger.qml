import QtQuick 2.4
import QtQuick.Controls 2.4
import QtQuick.Layouts 1.0
import SofaBasics 1.0
import SofaApplication 1.0
import QtCharts 2.3
import QtGraphicalEffects 1.0

SofaCanvas {
    id: root
    property var cableController: SofaApplication.sofaScene.get("/finger/cable/aCableActuator")
    property var scene: SofaApplication.sofaScene

    function keyPressed(event) {
        console.log(event)
        console.log("KeyPressed in finger.qml")
    }

    width: 40
    height: 20
    color: "transparent"
    ColumnLayout {
        anchors.fill: parent
        spacing: 10

        Slider {
            from: 0
            to: 40
            value: 0
            onValueChanged: {
                cableController.getData("value").value = [value]
            }
        }

        Button {
            text: "-"
            width: 20
            onClicked: {
                var val = Number(cableController.getData("value").value[0])
                val -= 1.0
                console.log([val])
                cableController.getData("value").value = [val]
            }
        }
        Button {
            text: "+"
            width: 20
            onClicked: {
                var val = Number(cableController.getData("value").value[0])
                val += 1.0
                console.log([val])
                cableController.getData("value").value = [val]
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
