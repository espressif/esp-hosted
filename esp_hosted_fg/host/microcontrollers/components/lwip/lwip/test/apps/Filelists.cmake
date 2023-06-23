# This file is indended to be included in end-user CMakeLists.txt
# include(/path/to/Filelists.cmake)
# It assumes the variable LWIP_DIR is defined pointing to the
# root path of lwIP sources.
#
# This file is NOT designed (on purpose) to be used as cmake
# subdir via add_subdirectory()
# The intention is to provide greater flexibility to users to 
# create their own targets using the *_SRCS variables.

set(LWIP_TESTDIR ${LWIP_DIR}/test/apps)
set(LWIP_TESTFILES
	${LWIP_TESTDIR}/test_apps.c
	${LWIP_TESTDIR}/test_sockets.c
	${LWIP_TESTDIR}/linux/sys_arch.c
)
