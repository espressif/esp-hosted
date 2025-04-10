# Iperf Core Engine

This repository contains a basic iperf core engine.

## Documentation

- There are two basic function:
  - `iperf_start`
  - `iperf_stop`


### Installation

- To add a plugin command or any component from IDF component manager into your project, simply include an entry within the `idf_component.yml` file.

  ```yaml
  dependencies:
    espressif/iperf:
      version: "^0.1.3"
  ```
- For more details refer [IDF Component Manager](https://docs.espressif.com/projects/idf-component-manager/en/latest/)
