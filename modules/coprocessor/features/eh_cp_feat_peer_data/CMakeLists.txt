# SPDX-License-Identifier: Apache-2.0
# eh_cp_feat_peer_data — peer data transfer extension
#
# Provides per-msg-id callback registration with local_context and
# send API. User opts in by calling
# eh_cp_feat_peer_data_init() at app level.


add_library(eh_cp_feat_peer_data STATIC
    "src/eh_cp_feat_peer_data.c"
)

target_include_directories(eh_cp_feat_peer_data
    PUBLIC
        "include"
)

target_link_libraries(eh_cp_feat_peer_data
    PUBLIC
        eh_cp_core
)
