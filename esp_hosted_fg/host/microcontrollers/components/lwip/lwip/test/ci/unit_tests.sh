#!/bin/bash
set -e

export CI_ROOT_DIR=`pwd`
export LWIPDIR=../../../../src
export ORIG_CC=${CC}
pushd `pwd`
cd ${CONTRIB}/ports/unix/check

### with GNU Make

# build and run default lwip tests (ESP_LWIP=0!)
make clean
make -j 4 check
# retest with ESP_LWIP patches
make clean
export EXTRA_CFLAGS="-DESP_LWIP=1" && export CC="${ORIG_CC} ${EXTRA_CFLAGS}"
make -j 4 check
# retest with IP_FORWARD enabled
make clean
export EXTRA_CFLAGS="-DESP_LWIP=1 -DIP_FORWARD=1" && export CC="${ORIG_CC} ${EXTRA_CFLAGS}"
make -j 4 check
# retest with IP_FORWARD and IP_NAPT enabled
make clean
export EXTRA_CFLAGS="-DESP_LWIP=1 -DIP_FORWARD=1 -DIP_NAPT=1" && export CC="${ORIG_CC} ${EXTRA_CFLAGS}"
make -j 4 check
# Please uncomment the below to test IP_FORWARD/IP_NAPT tests with debug output (only ip4_route test suite will be executed)
make clean
export EXTRA_CFLAGS="-DESP_LWIP=1 -DIP_FORWARD=1 -DESP_TEST_DEBUG=1 -DIP_NAPT=1" && export CC="${ORIG_CC} ${EXTRA_CFLAGS}"
make -j 4 check


### with CMake
cd ${CI_ROOT_DIR}/${CONTRIB}/ports/unix/check
rm -rf build
export EXTRA_CFLAGS=""
export CC="${ORIG_CC}"
mkdir build && cd build && cmake -DLWIP_DIR=`pwd`/../../../../.. .. -G Ninja
cmake --build . && ./lwip_unittests
[ -f ${CI_ROOT_DIR}/check2junit.py ] &&
    python ${CI_ROOT_DIR}/check2junit.py lwip_unittests.xml > ${CI_ROOT_DIR}/unit_tests.xml

popd
