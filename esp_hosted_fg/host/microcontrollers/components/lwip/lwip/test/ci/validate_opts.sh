#!/bin/bash
set -e

export CFLAGS="-I../../../../test/unix -include esp_lwipopts.h"
export LWIPDIR=../../../../src/
cp ${CONTRIB}/examples/example_app/lwipcfg.h.example ${CONTRIB}/examples/example_app/lwipcfg.h
cd ${CONTRIB}/ports/unix/example_app

make TESTFLAGS="-Wno-documentation" -j 4
chmod +x iteropts.sh
./iteropts.sh
