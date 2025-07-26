/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

struct sbgc_encoder_class_s;

typedef struct sbgc_encoder_s {
    struct sbgc_encoder_class_s *cls;
} sbgc_encoder;

typedef struct sbgc_encoder_class_s {
    int32_t (*read)(sbgc_encoder *enc);
    void (*free)(sbgc_encoder *enc);
    uint32_t scale;
} sbgc_encoder_class;

#endif /* ENCODER_H */
