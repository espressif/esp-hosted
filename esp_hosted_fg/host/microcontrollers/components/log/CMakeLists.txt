idf_build_get_property(target IDF_TARGET)
set(srcs "esp_log.c" )
set(priv_requires "")

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")

