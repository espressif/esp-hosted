# Espressif Wireless Framework

## Introduction

This project is used to build firmware for esp-hosted-ng solution

## Update build firmware for esp-hosted-ng solution

1. run ./setup.sh to setup enviornment, it will setup esp-idf as submodule to be used by `network_adapter`

2. setup compiling environment by `. ./export.sh` in esp-idf directory

3. in the `network_adapter` directory of this project, input command `idf.py build` to recompile `network_adapter` to generate new firmware.
# esp-hosted-80211-fw

