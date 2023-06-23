#!/bin/bash
set -e

export LWIP_DIR=`pwd`
export LWIP_CONTRIB_DIR=`pwd`/${CONTRIB}

cd test/apps
# Prepare a failing report in case we get stuck (check in no-fork mode)
python socket_linger_stress_test.py failed > ${LWIP_DIR}/socket_linger_stress_test.xml
for cfg in config_no_linger config_linger config_linger_reuse; do
    cmake -DCI_BUILD=1 -DTEST_CONFIG=${cfg} -B ${cfg} -G Ninja .
    cmake --build ${cfg}/
    timeout 10 ./${cfg}/lwip_test_apps
    [ -f check2junit.py ] &&
        python ${LWIP_DIR}/check2junit.py lwip_test_apps.xml > ${LWIP_DIR}/${cfg}.xml
done
# Run the lingering test multiple times
for run in {1..10000}; do ( timeout 10 ./config_linger/lwip_test_apps ) || exit 1 ; done
# All good, regenerate the stress test-report, since the test succeeded
python socket_linger_stress_test.py > ${LWIP_DIR}/socket_linger_stress_test.xml
