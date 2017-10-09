#!/bin/bash

# Script to generate SRC_FILES and INC_FOLDERS in Makefile
#
# Require:
# 		app_config.h
#
# Output:
# 		SRC_FILES
#		INC_FOLDERS
#
# Install location:
#		Same place as Makefile
#

SDK_ROOT="../../../../../.."
APP_CONF=`pwd`/../config/app_config.h
SRC_FILE=""
INC_PATH=""
FILES=`awk '/^#define.*ENABLED/ {print tolower($2)}' $APP_CONF | sed -e 's/_enabled*/.c/g'`
for i in $FILES; do
	j=$i
	HAS_=`echo $i | egrep '_' - ; echo $?`
	if [ "$HAS_" == "1" ]; then
		i="nrf_drv_$i"
	fi
	CPATH=`find $SDK_ROOT -name $i -print`
	if [ "$CPATH" == "" ]; then
		CPATH=`find $SDK_ROOT -name $j -print`
	fi
	if [ "$CPATH" == "" ]; then
		CPATH=`find $SDK_ROOT -name "app_$j" -print`
	fi
	if [ "$CPATH" != "" ]; then
		SRC_FILE="${SRC_FILE} `echo $CPATH | sed -e \"s#^$SDK_ROOT#\$\(SDK_ROOT\)#\"`"
		INC_PATH="${INC_PATH} `dirname $CPATH | sed -e \"s#^$SDK_ROOT#\$\(SDK_ROOT\)#\"`"
		# echo "Found $i"
	else
		echo "WARNING: $j not found"
	fi
done
echo "SRC_FILES += \\"
for i in $SRC_FILE; do
	echo "$i \\"
done
echo "INC_FOLDERS += \\"
for i in $INC_PATH; do
	echo $i
done


