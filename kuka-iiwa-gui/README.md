# kuka-iiwa-gui

NISHI, Takao <zophos@ni.aist.go.jp>

GUI for [KUKA-IIWA-API](https://github.com/jonaitken/KUKA-IIWA-API)

## Installation

### Dependency

 + [ROS](http://www.ros.org/)
 + [KUKA-IIWA-API](https://github.com/jonaitken/KUKA-IIWA-API)
 + [node.js](https://nodejs.org/)
   + electron
   + rosnodejs
   + @fontawesome/fontawesome-free
   + roboto-fontface

 1. install node.js and npm
 2. run `npm install` to install electron, rosnodejs, fontawesome-free and roboto-fontface

## Running

 1. run roscore
 2. run kuka_iiwa_api `rosrun kuka_controller server_V30032017.py`
 3. run kuka-iiwa-gui `./node_modules/.bin/electron .`

## Operation

TBW
