# Common components root

add_subdirectory(eh_common ${CMAKE_BINARY_DIR}/eh_common)
add_subdirectory(eh_frame ${CMAKE_BINARY_DIR}/eh_frame)
add_subdirectory(eh_mempool ${CMAKE_BINARY_DIR}/eh_mempool)
add_subdirectory(eh_tlv ${CMAKE_BINARY_DIR}/eh_tlv)

# Serializers (pure CMake)
add_subdirectory(serializers/eh_proto_linux_v1 ${CMAKE_BINARY_DIR}/eh_proto_linux_v1)
add_subdirectory(serializers/eh_proto_mcu_v1 ${CMAKE_BINARY_DIR}/eh_proto_mcu_v1)
add_subdirectory(serializers/eh_proto_v2 ${CMAKE_BINARY_DIR}/eh_proto_v2)
add_subdirectory(serializers/third_party/msg_codec ${CMAKE_BINARY_DIR}/msg_codec)
