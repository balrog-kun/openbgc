/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef PARAMS_H
#define PARAMS_H

struct obgc_param_s {
    uint16_t id;
    uint8_t size;
    uint8_t flags;
    /* Is it better to have a single pointer here, and point to an extra struct if pointer
     * or setter is necessary, or have these optional (potentially unused) extra fields here?
     */
    union {
        void *addr;
        const void *addr_ro;
        uint32_t ptr_offset;
    };
    union {
        uint32_t unused;
        void **ptr;
        void *setter; /* Might need both ptr and setter but until we do... */
    };
};

enum obgc_param_flag_num_e {
    param_flag_store,
    param_flag_ro,
    param_flag_ptr,
    param_flag_setter,
};

extern const struct obgc_param_s params[];

void *params_data_ptr(const struct obgc_param_s *param);

#endif /* PARAMS_H */
