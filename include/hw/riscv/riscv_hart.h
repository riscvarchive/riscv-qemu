#ifndef HW_RISCV_HART_H
#define HW_RISCV_HART_H

#define TYPE_RISCV_HART_ARRAY "riscv.hart_array"

#define RISCV_HART_ARRAY(obj) \
    OBJECT_CHECK(RISCVHartArrayState, (obj), TYPE_RISCV_HART_ARRAY)

typedef struct RISCVHartArrayState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    uint32_t num_harts;
    char *cpu_model;
    RISCVCPU *harts;
} RISCVHartArrayState;

#endif
