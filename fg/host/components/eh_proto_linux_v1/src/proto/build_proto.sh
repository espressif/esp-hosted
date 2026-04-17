#!/usr/bin/env bash

which protoc-c > /dev/null
if [ $? -ne 0 ]; then
	echo "Please install protoc-c"
	exit 1
fi

protoc-c ./esp_hosted_config.proto --c_out=.
if [ $? -ne 0 ]; then
	echo "Build failed, Please review esp_hosted_config.proto"
	exit 1
fi

mv esp_hosted_config.pb-c.h ../include/esp_hosted_config.pb-c.h
mv esp_hosted_config.pb-c.c ../esp_hosted_config.pb-c.c

echo "Generated protobuf definitions successfully"
