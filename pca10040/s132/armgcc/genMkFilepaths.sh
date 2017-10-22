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
SRC_FILES=""
INC_FOLDERS=""
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
#		SRC_FILE="${SRC_FILE} `echo $CPATH | sed -e \"s#^$SDK_ROOT#\$\(SDK_ROOT\)#\"`"
#		INC_FOLDERS="${INC_FOLDERS} `dirname $CPATH | sed -e \"s#^$SDK_ROOT#\$\(SDK_ROOT\)#\"`"
		SRC_FILE=`echo $CPATH | sed -e "s#^$SDK_ROOT#\$\(SDK_ROOT\)#"`
		INC_FOLDER=`dirname $CPATH | sed -e "s#^$SDK_ROOT#\$\(SDK_ROOT\)#"`
		if ! fgrep -q "$SRC_FILE" Makefile; then
			SRC_FILES="${SRC_FILES} $SRC_FILE"
		fi
		if ! fgrep -q "$INC_FOLDERS" Makefile; then
			INC_FOLDERS="${INC_FOLDERSS} $INC_FOLDER"
		fi
		# echo "Found $i"
	else
		echo "WARNING: $j not found"
	fi
done
echo "SRC_FILES += \\"
for i in $SRC_FILES; do
	echo "$i \\"
done
echo "INC_FOLDERS += \\"
for i in $INC_FOLDERS; do
	echo $i
done


