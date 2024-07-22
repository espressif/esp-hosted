#!/bin/bash

# Ensure script stops on errors
set -e

TGT_NAME=$1

# Navigate to the target directory
cd esp_hosted_ng/esp/esp_driver/

echo "ESP hosted: initializing submodule esp-idf"
git submodule update --init --depth=1

echo "ESP hosted: initializing submodule for esp-idf"
cd esp-idf
git submodule update --init --depth=1

echo "ESP hosted: installing prerequisites for $TGT_NAME"
./install.sh $TGT_NAME
cd ..

echo "ESP hosted: replacing wireless libraries"
rm -rf esp-idf/components/esp_wifi/lib/*
cp -r lib/* esp-idf/components/esp_wifi/lib/

echo "###### Setup Done ######"

cd esp-idf
echo "Exporting variables"
. ./export.sh

cd ../network_adapter

# Check if the second argument (assumed to be a string) matches "spi"
if [ "$2" = "spi" ]; then
    # Check if sdkconfig.ci exists
    if [ -f "sdkconfig.ci" ]; then
        # Append the content of sdkconfig.ci to sdkconfig.defaults
        echo "appending ci config to default"
        cat sdkconfig.ci >> sdkconfig.defaults
    else
        echo "Error: sdkconfig.ci does not exist."
        exit 1
    fi
fi

echo "Setting target as $TGT_NAME"
idf.py set-target $TGT_NAME
idf.py build

# Check if the build was successful
if [ ! -f "build/network_adapter.bin" ]; then
    echo "Compilation failed; exit 1"
    exit 1
fi

echo "Build successful"
