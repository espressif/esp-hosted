#!/bin/bash

# This script copies and touches the .project, .cproject and .mxproject files
# This is required to make STM32CubeIDE recognize the new project import

PROJ_NAME=stm_spi_host
CODE_BASE=

function usage() {
	echo "$0 <Workspace directory>"
}

function copy_and_touch_files() {
	CWD=`pwd`
	cd ../../../
	CODE_BASE=`pwd`
	cd $CWD
	cp ./.project $WORKSPACE/$PROJ_NAME/
	cp ./.cproject $WORKSPACE/$PROJ_NAME/
	sed -i.bak "s#CODE_BASE_PLACE_HOLDER#$CODE_BASE#" $WORKSPACE/$PROJ_NAME/.project
	sed -i.bak "s#CODE_BASE_PLACE_HOLDER#$CODE_BASE#" $WORKSPACE/$PROJ_NAME/.cproject
	touch $WORKSPACE/$PROJ_NAME/.mxproject
	echo "success. Now, please open STM32CubeIDE with $WORKSPACE"
}

if [ "$#" != "1" ] ; then
	echo "Err: Workspace directory created for STM32CubeIDE to be passed as argument"
	usage
	exit 1;
else
	WORKSPACE=$1
fi

if [ "$1" == '-h' ] || [ "$1" == '--help' ] ; then
	usage
	exit 0
fi


if [ ! -d $WORKSPACE ]; then
	echo "Err: $WORKSPACE directory not found. Please follow documentation to import STM project from stm_spi_host_<ESP_slave_board_type>.ioc first"
	usage
	exit 1;
fi

if [ ! -d $WORKSPACE/$PROJ_NAME ]; then
	echo "Err: Either incorrect Workspace directory or ioc project not imported."
	echo "Please follow documentation to import STM project from stm_spi_host_<ESP_slave_board_type>.ioc if not already done"
	usage
	exit 1;
fi

copy_and_touch_files
