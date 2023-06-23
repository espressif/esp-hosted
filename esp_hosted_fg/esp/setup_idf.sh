#!/usr/bin/env bash
#Helper script for kenwood building

FW_SETUP_DIR=esp-hosted/esp_hosted_fg/esp

#colorful steps
COLOR_ECHO=1


setup_idf()
{
	prompt_string "Setup IDF"
	pushd .
	git submodule update --init --recursive
	rm -rf ~/.espressif
	cd $FW_SETUP_DIR
	cmake .
	export IDF_PATH="$(pwd)/esp-idf"
	. esp-idf/export.sh
	popd > /dev/null

	command -v idf.py
	if [ "$?" != "0" ]; then
		echo "idf.py not found"
		exit 1
	fi
}

prompt_string()
{
	if [ "$COLOR_ECHO" != "" ] ; then
		RED='\033[0;31m'
		NC='\033[0m' # No Color
	fi

	if [ "$1" != "" ] ; then
		str=$1
		echo -e "$RED"
		echo "*************************************"
		echo "     $str"
		echo "*************************************"
		echo -e "$NC"
	fi
}


if [ -z "$BASH" ]; then echo "Please run this script $0 with bash"; exit; fi
setup_idf
