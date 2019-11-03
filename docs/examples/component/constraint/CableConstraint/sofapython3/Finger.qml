import QtQuick 2.4
import QtQuick.Controls 2.4
import QtQuick.Layouts 1.0
import SofaBasics 1.0
import SofaApplication 1.0

Rectangle {
    id: root
    property var cableController: null
    width: 40
    height: 20
    color: "transparent"
    RowLayout {
        anchors.fill: parent

        Button {
            text: "-"
            width: 20
            onClicked: {
                if (cableController === null)
                    cableController = SofaApplication.sofaScene.get("/finger/cable/aCableActuator")
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
                if (cableController === null)
                    cableController = SofaApplication.sofaScene.get("/finger/cable/aCableActuator")
                var val = Number(cableController.getData("value").value[0])
                val += 1.0
                console.log([val])
                cableController.getData("value").value = [val]
            }
        }
    }
}
