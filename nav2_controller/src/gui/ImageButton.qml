// Copyright (c) 2018 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import QtQuick 2.5

Rectangle {
  id: container
  property alias text: buttonText.text
  property alias imagePath: buttonImage.source
  signal clicked()
  width: parent.width
  height: 2.8 * itemHeight
  color: "transparent"

  MouseArea {
    anchors.fill: parent
    hoverEnabled: true

    onClicked: {
      container.clicked() 
    }

    onEntered: {
      container.color = "#333333"
      padding1.color = "#333333"
      padding2.color = "#333333"
      buttonTextContainer.color = "#333333"
    }

    onExited: {
      container.color = "transparent"
      padding1.color = "transparent"
      padding2.color = "transparent"
      buttonTextContainer.color = "transparent"
    }
  }

  Rectangle {
    id: padding1
    color: "transparent"
    height: itemHeight * 0.2
    width: parent.width
    anchors {
    }
  }

  Image {
    id: buttonImage
    source: "qrc:/images/layout.png"
    anchors.margins: 10
    anchors.horizontalCenter: parent.horizontalCenter
    anchors.top: padding1.bottom
    width: mainWindow.itemHeight * 1.8   // container.width/2
    height: mainWindow.itemHeight * 1.8 // container.width/2
  }

  Rectangle {
    id: buttonTextContainer
    color: "transparent"
    height: itemHeight
    width: parent.width
    anchors {
      top: buttonImage.bottom
    }

    MouseArea {
      anchors.fill: parent
      onClicked: {
        container.clicked()
      }
    }
    Text {
      id: buttonText
      text: "ImageButton text"
      font.pixelSize: mainWindow.fontSize * 0.75
      color: "#A0A0A0"
      horizontalAlignment: Text.AlignHCenter
      verticalAlignment: Text.AlignVCenter
      anchors.fill: parent
    }
  }

  Rectangle {
    id: padding2
    color: "transparent"
    height: itemHeight * 0.2
    width: parent.width
    anchors {
      top: buttonTextContainer.bottom
    }
  }
}
