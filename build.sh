#!/bin/bash

export QT_QPA_PLATFORM=xcb

rm -rf ~/RP3/ThinWalls/build/
mkdir ~/RP3/ThinWalls/build/
cd ~/RP3/ThinWalls/build/
cmake ..
make
