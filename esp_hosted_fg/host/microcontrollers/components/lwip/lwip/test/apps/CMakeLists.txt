cmake_minimum_required(VERSION 3.8)

project(lwip_test_apps C)

if (NOT CMAKE_SYSTEM_NAME STREQUAL Linux AND NOT CMAKE_SYSTEM_NAME STREQUAL Darwin)
    message(FATAL_ERROR "Unit test are currently only working on Linux or Darwin")
endif()

if (NOT LWIP_DIR OR NOT LWIP_CONTRIB_DIR)
    set(LWIP_DIR "$ENV{LWIP_DIR}")
    set(LWIP_CONTRIB_DIR "$ENV{LWIP_CONTRIB_DIR}")
endif()

if (NOT CI_BUILD)
    set(LWIP_USE_SANITIZERS true)
endif()

include(${LWIP_CONTRIB_DIR}/ports/CMakeCommon.cmake)

if(CMAKE_C_COMPILER_ID STREQUAL Clang)
    # check.h causes 'error: token pasting of ',' and __VA_ARGS__ is a GNU extension' with clang 9.0.0
endif()

set (LWIP_DEFINITIONS -DLWIP_DEBUG -DLWIP_NOASSERT_ON_ERROR -DLWIP_OPTTEST_FILE)
set (LWIP_INCLUDE_DIRS
    "${LWIP_DIR}/test/apps"
    "${LWIP_DIR}/test/unix"
    "${LWIP_CONTRIB_DIR}/ports/unix/port/include"
    "${LWIP_DIR}/test/unit"
    "${LWIP_DIR}/src/include"
    "${LWIP_CONTRIB_DIR}/"
    "${CMAKE_CURRENT_SOURCE_DIR}/"
    "${CMAKE_CURRENT_SOURCE_DIR}/${TEST_CONFIG}"
)

include(${LWIP_CONTRIB_DIR}/ports/unix/Filelists.cmake)
include(${LWIP_DIR}/src/Filelists.cmake)
include(${LWIP_DIR}/test/apps/Filelists.cmake)

add_executable(lwip_test_apps ${LWIP_TESTFILES})
target_include_directories(lwip_test_apps PRIVATE ${LWIP_INCLUDE_DIRS})
target_compile_options(lwip_test_apps PRIVATE ${LWIP_COMPILER_FLAGS})
target_compile_definitions(lwip_test_apps PRIVATE ${LWIP_DEFINITIONS} ${LWIP_MBEDTLS_DEFINITIONS})

find_library(LIBCHECK check)
find_library(LIBM m)
target_link_libraries(lwip_test_apps ${LWIP_SANITIZER_LIBS} lwipallapps lwipcore ${LIBCHECK} ${LIBM})

if (NOT CMAKE_SYSTEM_NAME STREQUAL Darwin)
    # check installed via brew on Darwin doesn't have a separate subunit library (must be statically linked)
    find_library(LIBSUBUNIT subunit)
    target_link_libraries(lwip_test_apps ${LIBSUBUNIT})
endif()

if (CMAKE_SYSTEM_NAME STREQUAL Linux)
    find_library(LIBUTIL util)
    find_library(LIBPTHREAD pthread)
    find_library(LIBRT rt)
    target_link_libraries(lwip_test_apps ${LIBUTIL} ${LIBPTHREAD} ${LIBRT})
endif()

if (CMAKE_SYSTEM_NAME STREQUAL Darwin)
    # Darwin doesn't have pthreads or POSIX real-time extensions libs
    find_library(LIBUTIL util)
    target_link_libraries(lwip_test_apps ${LIBUTIL})
endif()
