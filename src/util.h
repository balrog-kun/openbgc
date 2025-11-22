/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef UTIL_H
#define UTIL_H

#ifdef __cplusplus
# define __auto_type auto
#endif

#define container_of(ptr, type, member) ((type *) ((char *) (ptr) - offsetof(type, member)))

#define STRINGIFY(val) STRINGIFY_ARG(val)
#define STRINGIFY_ARG(contents) #contents

#define AUTO_FREE_VAR(vartype, varname) vartype varname __attribute__((cleanup(auto_free)))

#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

#define steal_ptr(ptr) (__extension__ ({ typeof(ptr) _tmp = (ptr); (ptr) = NULL; _tmp; }))
#define steal_num(num) (__extension__ ({ typeof(num) _tmp = (num); (num) = 0; _tmp; }))

#define _IN_SET_CMP(val, type, cmp, ...) __extension__ ({ \
        const type __v = (val); \
        const typeof(__v) __elems[] = {__VA_ARGS__}; \
        unsigned int __i; \
        static const unsigned int __n = ARRAY_SIZE(__elems); \
        bool __r = false; \
        for (__i = 0; __i < __n && !__r; __i++) \
            __r = (cmp); \
        __r; \
    })

/* Warning: evaluates all set elements even after @val has matched one */
#define IN_SET(val, ...) _IN_SET_CMP((val), __auto_type, __v == __elems[__i], ##__VA_ARGS__)

#define IN_STRSET(val, ...) \
    _IN_SET_CMP((val), char *, __v == __elems[__i] || (__v && __elems[__i] && !strcmp(__v, __elems[__i])), ##__VA_ARGS__)

#if __STDC_VERSION__ >= 201112L
# define CHECK_SIZE(struct_type, max_size) \
    _Static_assert(sizeof(struct struct_type) <= (max_size), #struct_type " too large")
#elif defined(__cpp_static_assert)
# define CHECK_SIZE(struct_type, max_size) \
    static_assert(sizeof(struct struct_type) <= (max_size), #struct_type " too large")
#else
# define CHECK_SIZE(struct_type, max_size) \
    typedef char __check_##struct_type##_size[(sizeof(struct_type) <= (max_size)) ? 1 : -1] __attribute__((unused))
#endif

#define clamp constrain /* Better name */

#endif /* UTIL_H */
