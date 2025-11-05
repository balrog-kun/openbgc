/* vim: set ts=4 sw=4 sts=4 et : */
#ifndef MAIN_H
#define MAIN_H

struct main_loop_cb_s {
    void *data; /* Just a favor to the users, they could as well use container_of() if space was an issue */
    void (*cb)(void *data);
    struct main_loop_cb_s *next;
};

void main_loop_sleep(void);
void main_loop_cb_add(struct main_loop_cb_s *cb);
void main_loop_cb_remove(struct main_loop_cb_s *cb);

#ifdef __cplusplus
extern HardwareSerial *error_serial;
#endif
void error_serial_print(const char *func, const char *msg);
#define error_print(msg) error_serial_print(__func__, (msg))

#endif /* MAIN_H */
