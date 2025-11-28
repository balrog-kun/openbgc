/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "moremath.h"

struct obgc_encoder_class_s;

typedef struct obgc_encoder_s {
    struct obgc_encoder_class_s *cls;
    int32_t reading_raw;
    float reading, reading_rad;
} obgc_encoder;

typedef struct obgc_encoder_class_s {
    int32_t (*read)(obgc_encoder *enc);
    void (*free)(obgc_encoder *enc);
    uint32_t scale;
    float resolution;
} obgc_encoder_class;

static inline void encoder_update(struct obgc_encoder_s *enc) {
    if (!enc)
        return;

    enc->reading_raw = enc->cls->read(enc);
    enc->reading = (float) enc->reading_raw / enc->cls->scale;
    enc->reading_rad = enc->reading_raw * (D2R / enc->cls->scale);
}

#endif /* ENCODER_H */
