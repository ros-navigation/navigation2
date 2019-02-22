import QtQuick 2.5
import QtQuick.Window 2.2
import QtQuick.Controls 1.4

ApplicationWindow {

  id: mainWindow
  color: "#101010"

  // Scalability parameters
  property int pixDens: Math.ceil(Screen.pixelDensity)
  property int itemWidth: 25 * pixDens
  property int itemHeight: 10 * pixDens
  property int scaledMargin: 2 * pixDens
  property int fontSize: 5 * pixDens

  visible: true
  width: 145
  height: 260
  title: "Nav2 Controller"
  Component.onCompleted: {
    setX(1);
    setY(1);
  }

  Text {
    anchors.fill: parent
    text:  obj.text
    font.pointSize: 18
    font.bold: true
  }

  Rectangle {
    id: imageButtons
    color: "#041F48"
    anchors.fill: parent

    Column {
      anchors {
        right: parent.right
        left: parent.left
      }

      spacing: itemHeight/4

      ImageButton {
        objectName: "qml_rectangle"
        text: "Startup/Shutdown"
        //imagePath: "qrc:/images/layout.png"
        imagePath: "images/Play.png"
        onClicked: {
        }
      }

      ImageButton {
        objectName: "qml_rectangle2"
        text: "Pause/Resume"
        //imagePath: "qrc:/images/settings.png"
        imagePath: "images/Pause.png"
        onClicked: {
        }
      }
    }
  }
}
