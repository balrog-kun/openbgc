/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MAIN_H
#define MAIN_H

struct main_loop_cb_s {
    void *data;
    void (*cb)(void *data);
    struct main_loop_cb_s *next;
};

void main_loop_sleep(void);
void main_loop_cb_add(struct main_loop_cb_s *cb);
void main_loop_cb_remove(struct main_loop_cb_s *cb);

#endif /* MAIN_H */
