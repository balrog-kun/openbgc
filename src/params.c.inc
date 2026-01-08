/* vim: set ts=4 sw=4 sts=4 et : */

#define PARAM_DEF(data)                 PARAM_DEF_FULL(.addr = &data,, data, 0)
#define PARAM_DEF_SAVE(data)            PARAM_DEF_FULL(.addr = &data,, data, 1 << param_flag_store)
#define PARAM_DEF_RO(data)              PARAM_DEF_FULL(.addr_ro = &data,, data, 1 << param_flag_ro)
#define PARAM_DEF_PTR(_ptr, offset)     PARAM_DEF_FULL(.ptr_offset = offsetof(typeof(*_ptr), offset), \
    .ptr = (void **) &_ptr, _ptr->offset, 1 << param_flag_ptr)
#define PARAM_DEF_PTR_RO(_ptr, offset)  PARAM_DEF_FULL(.ptr_offset = offsetof(typeof(*_ptr), offset), \
    .ptr = (void **) &_ptr, _ptr->offset, (1 << param_flag_ptr) | (1 << param_flag_ro))
#define PARAM_DEF_SETTER(data, _setter) PARAM_DEF_FULL(.addr = &data, .setter = _setter, data, 1 << param_flag_setter)
#define PARAM_DEF_CONST(value)          PARAM_DEF_FULL(.val = (value),, uint32_t, \
    (1 << param_flag_const) | (1 << param_flag_ro))

/* Note: line is already a number so we could use it as the ID directly, but the enum makes the
 * IDs independent of the line numbers and only dependent on the definition order.
 */
#define PARAM_DEF_FULL(addr_def, ptr_def, data, flags) CONCAT(param_, __LINE__),
#define PARAM_UNUSED(data) PARAM_DEF_FULL(NULL, NULL, data, 0)

enum obgc_param_id_e {
#include "param-defs.c.inc"
    param_max_id
} param_ids;

#undef PARAM_UNUSED
#undef PARAM_DEF_FULL

#define PARAM_DEF_FULL(addr_def, ptr_def, data, _flags) { \
        .id = CONCAT(param_, __LINE__), \
        .size = sizeof(data), \
        .flags = _flags, \
        addr_def, \
        ptr_def \
    },
#define PARAM_UNUSED(data)

const struct obgc_param_s params[] = {
#include "param-defs.c.inc"
};

#undef PARAM_UNUSED
#undef PARAM_DEF_FULL

const void *param_data_const_ptr(const struct obgc_param_s *param) {
    if (param->flags & (1 << param_flag_ptr)) {
        if (!*param->ptr)
            return NULL;

        return (uint8_t *) *param->ptr + param->ptr_offset;
    } else if (param->flags & (1 << param_flag_const))
        return &param->val;
    else
        return param->addr;
}

void *param_data_ptr(const struct obgc_param_s *param) {
    if (param->flags & (1 << param_flag_ro))
        return NULL;
    else
        return (void *) param_data_const_ptr(param);
}
