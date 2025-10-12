/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

struct obgc_encoder_class_s;

typedef struct obgc_encoder_s {
    struct obgc_encoder_class_s *cls;
} obgc_encoder;

typedef struct obgc_encoder_class_s {
    int32_t (*read)(obgc_encoder *enc);
    void (*free)(obgc_encoder *enc);
    uint32_t scale;
} obgc_encoder_class;

#endif /* ENCODER_H */
