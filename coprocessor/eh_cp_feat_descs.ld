SECTIONS
{
    .eh_cp_feat_descs_dram : ALIGN(4)
    {
        _eh_cp_feat_descs_start = .;
        KEEP(*(.eh_cp_feat_descs_dram))
        _eh_cp_feat_descs_end = .;
    } > sram_seg
} INSERT AFTER .dram0.data
