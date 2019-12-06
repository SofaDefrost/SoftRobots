import QtQuick 2.4
import QtQuick.Controls 2.4
import QtQuick.Layouts 1.0
import SofaBasics 1.0
import SofaApplication 1.0

Rectangle {
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
    RowLayout {
        anchors.fill: parent

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
    }
}
