#!/bin/bash

# This script copies and touches the .project, .cproject and .mxproject files
# This is required to make STM32CubeIDE recognize the new project import

PROJ_NAME=
CODE_BASE=

function usage() {
	echo "$0 <Transport> <Workspace_directory>"
	echo "Transport - either spi or sdio"
	echo "Workspace_directory - Workspace directory created for STM32CubeIDE"
}

function copy_and_touch_files() {
	CWD=`pwd`
	cd ../../../
	CODE_BASE=`pwd`
	cd $CWD
	cp ./${TRANSPORT}/.project $WORKSPACE/$PROJ_NAME/
	cp ./${TRANSPORT}/.cproject $WORKSPACE/$PROJ_NAME/
	sed -i.bak "s#CODE_BASE_PLACE_HOLDER#$CODE_BASE#" $WORKSPACE/$PROJ_NAME/.project
	sed -i.bak "s#CODE_BASE_PLACE_HOLDER#$CODE_BASE#" $WORKSPACE/$PROJ_NAME/.cproject
	touch $WORKSPACE/$PROJ_NAME/.mxproject
	echo "success. Now, please open STM32CubeIDE with $WORKSPACE"
}

if [ "$1" == '-h' ] || [ "$1" == '--help' ] ; then
	usage
	exit 0
fi

if [ "$#" != "2" ] ; then
	echo "Err: Incorrect number of arguments passed"
	usage
	exit 1
else
	if [ "$1" != "spi" -a "$1" != "sdio" ] ; then
		echo "ERR: Invalid transport value"
		usage
		exit 1
	fi
	TRANSPORT=$1
	WORKSPACE=$2
	PROJ_NAME=stm_${TRANSPORT}_host
fi



if [ ! -d $WORKSPACE ]; then
	echo "Err: $WORKSPACE directory not found. Please follow documentation to import STM project from stm_<transport>_host_<ESP_slave_board_type>.ioc first"
	echo "<ESP_slave_board_type> is applicable only for SPI transport"
	echo "SPI : for ESP32 wroom/wrover use stm_spi_host_v1.ioc "
	echo "	  : for other ESP modules use stm_spi_host_v2.ioc"
	echo "SDIO: for ESP32 use stm_sdio_host.ioc"
	usage
	exit 1;
fi

if [ ! -d $WORKSPACE/$PROJ_NAME ]; then
	echo "Err: Either incorrect Workspace directory or ioc project not imported."
	echo "Please follow documentation to import STM project from stm_<transport>_host_<ESP_slave_board_type>.ioc if not already done"
	echo "SPI : for ESP32 wroom/wrover use stm_spi_host_v1.ioc "
	echo "	  : for other ESP modules use stm_spi_host_v2.ioc"
	echo "SDIO: for ESP32 use stm_sdio_host.ioc"
	usage
	exit 1;
fi

copy_and_touch_files
