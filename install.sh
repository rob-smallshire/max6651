#!/bin/sh

# To use this installation script, modify the
# line below to point to the 'libraries'
# directory of your Arduino installation.
#
# The script must be run from inside this
# directory like this:
#
# sudo ./install.sh
#

TARGET_DIR=/opt/arduino/arduino-0022/libraries/Max6651

mkdir -p $TARGET_DIR/utility

cp Max6651.h $TARGET_DIR/Max6651.h
cp keywords.txt $TARGET_DIR/keywords.txt
cp utility/AbstractI2C.h $TARGET_DIR/utility/AbstractI2C.h
cp utility/ConfigureMax6651.h $TARGET_DIR/utility/ConfigureMax6651.h
cp utility/ConfigureMax6651.cpp $TARGET_DIR/utility/ConfigureMax6651.cpp
cp utility/I2CMaster.h $TARGET_DIR/utility/I2CMaster.h
cp utility/I2CMaster.cpp $TARGET_DIR/utility/I2CMaster.cpp
cp utility/WireLikeI2C.h $TARGET_DIR/utility/WireLikeI2C.h
cp utility/Max6651Constants.h $TARGET_DIR/utility/Max6651Constants.h
cp utility/Max6651ClosedLoop.h $TARGET_DIR/utility/Max6651ClosedLoop.h
cp utility/Max6651ClosedLoop.cpp $TARGET_DIR/utility/Max6651ClosedLoop.cpp
