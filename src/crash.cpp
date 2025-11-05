/* vim: set ts=4 sw=4 sts=4 et : */
#include <Arduino.h>

/* Override weak fault handlers defined by who? CMSIS? */
extern "C" void HardFault_Handler(void) {
    __asm volatile (
        "tst lr, #4 \n"
        "ite eq \n"
        "mrseq r0, msp \n"
        "mrsne r0, psp \n"
        "mov r1, #0 \n"
        "mov r2, r11 \n"
        "b crash_handler \n"
        ::: "r0", "r1"
    );
    while (1);
}

extern "C" void MemManage_Handler(void) {
    __asm volatile (
        "tst lr, #4 \n"
        "ite eq \n"
        "mrseq r0, msp \n"
        "mrsne r0, psp \n"
        "mov r1, #1 \n"
        "mov r2, r11 \n"
        "b crash_handler \n"
        ::: "r0", "r1"
    );
    while (1);
}

extern "C" void BusFault_Handler(void) {
    __asm volatile (
        "tst lr, #4 \n"
        "ite eq \n"
        "mrseq r0, msp \n"
        "mrsne r0, psp \n"
        "mov r1, #2 \n"
        "mov r2, r11 \n"
        "b crash_handler \n"
        ::: "r0", "r1"
    );
    while (1);
}

extern "C" void UsageFault_Handler(void) {
    __asm volatile (
        "tst lr, #4 \n"
        "ite eq \n"
        "mrseq r0, msp \n"
        "mrsne r0, psp \n"
        "mov r1, #3 \n"
        "mov r2, r11 \n"
        "b crash_handler \n"
        ::: "r0", "r1"
    );
    while (1);
}

/* TODO: use the usart number from error_serial */

static void crash_usart_write(USART_TypeDef *USARTx, uint8_t ch) {
    /* STM32F3-specific, F1/F4 would be slightly different */
    while (!(USARTx->ISR & USART_ISR_TXE));  /* Wait until TX buffer empty */
    USARTx->TDR = ch;                        /* Write byte */
}

static uint8_t crash_usart_read(USART_TypeDef *USARTx) {
    while (!(USARTx->ISR & USART_ISR_RXNE)); /* Wait until RX buffer not empty */
    return USARTx->RDR;
}

#define FLASH_SIZE 0x20000 /* 128k */
#define SRAM_SIZE  0x8000  /* 32k */

struct fault_frame_s {
    uint32_t r0, r1, r2, r3, r12;
    void *lr, *pc;
    uint32_t psr;
};

struct arm_eabi_frame_s {
    uint32_t r4, r5, r6, r7, r8, r9, r10, r11 /* fp */, lr;
};

extern uint32_t _sstack;  /* Stack start (linker symbol) */
extern uint32_t _estack;  /* Stack end (linker symbol) */

static bool is_ram(const void *ptr) {
    return (uint32_t) ptr >= SRAM_BASE && (uint32_t) ptr < SRAM_BASE + SRAM_SIZE;
}

static bool is_flash(const void *ptr) {
    return (uint32_t) ptr >= FLASH_BASE && (uint32_t) ptr < FLASH_BASE + FLASH_SIZE;
}

static bool is_stack(const void *sp_min, const void *ptr) {
    if ((uint32_t) ptr & 3)
        return false;
#if 0
    if (_sstack)
        return (uint32_t) ptr >= _sstack && (uint32_t) ptr < _estack;
#endif
    return ptr > sp_min && is_ram(ptr);
}

static bool is_code(const void *ptr) {
    /* TODO: check if within .text */
    if (!((uint32_t) ptr & 1)) /* Should be Thumb */
        return false;
    return is_flash(ptr);
}

static void more_regs(const struct fault_frame_s *sp) {
    char msg[200];
    char *p;
    sprintf(msg, "r0 = %08lx, r1 = %08lx, r2 = %08lx, r3 = %08lx, r12 = %08lx, sp = %p, psr = %08lx\r\n",
        sp->r0, sp->r1, sp->r2, sp->r3, sp->r12, sp, sp->psr);
    for (p = msg; *p; p++)
        crash_usart_write(USART1, *p);
}

static void bt_frame(int i, const void *sp, const void *pc, const char *comment) {
    char msg[100];
    const char *p;
    bool ram = is_ram(pc);
    bool flash = is_flash(pc);
    bool stack = is_stack(sp, pc);
    bool code = is_code(pc);

    sprintf(msg, "#%i: 0x%08lx %s%s%s\r\n", i, (uint32_t) pc & ~1, comment,
        ram ? "(ram)" : (flash ? "(flash)" : "(garbage)"),
        stack ? "(stack)" : (code ? "(code)" : ""));
    for (p = msg; *p; p++)
        crash_usart_write(USART1, *p);
}

extern void setup_end(void);
extern void loop_end(void);

static void bt(const struct fault_frame_s *sp, const void *fp, bool scan, int max) {
    char msg[200];
    const char *p;
    const void **sptr;
    int i = 0;

    bt_frame(i++, sp, sp->pc, "(exception pc)");
    bt_frame(i++, sp, sp->lr, "(exception lr)");
    /*
     * Note: lr points to the instruction after the call but we can't compensate for it easily
     * because some branches are 2- and some are 4-byte long.
     */

    if (!is_ram(sp))
        sp = (const struct fault_frame_s *) __builtin_frame_address(0);

    if (scan)
        goto scan;

    /* Note: fp should only contain anything useful when compiling without -fomit-frame-pointer.
     * In practice even without it the frame format isn't very consistent.  Maybe need to also add
     * -mapcs-frame.
     */
    if (!is_stack(sp, fp))
        fp = __builtin_frame_address(0);

    if (is_stack(sp, fp) && ((const void **) fp)[1] == sp->lr)
        fp = ((const void **) fp)[0];

    while (i < max) {
        const void *lr, *caller_fp;

        if (!is_stack(sp, fp))
            break;

        lr = ((const void **) fp)[1];
        caller_fp = ((const void **) fp)[0];

        if (!is_code(lr))
            break;

        if ((lr >= setup && lr < setup_end) || (lr >= loop && lr < loop_end)) {
            bt_frame(i++, sp, lr, (lr >= setup && lr < setup_end) ? "setup() " : "loop() ");
            return;
        }

        bt_frame(i++, sp, lr, "");

        if (caller_fp <= (const uint8_t *) fp - 4)
            break;

        fp = caller_fp;
    }

    if (i >= 4) /* Success, no need to scan */
        return;

scan:
    /* TODO: could also scan the stack to try find fp */
    for (p = "Not using fp, scanning\r\n"; *p; p++)
        crash_usart_write(USART1, *p);

    for (sptr = (const void **) (sp + 1); is_stack(sp, sptr) && i < max; sptr++) {
        if (!is_code(*sptr))
            continue;

        const void *lr = *sptr;

        if ((lr >= setup && lr < setup_end) || (lr >= loop && lr < loop_end)) {
            bt_frame(i++, sp, lr, (lr >= setup && lr < setup_end) ? "setup() " : "loop() ");
            return;
        }

        bt_frame(i++, sp, lr, "");
    }
}

extern "C" void crash_handler(const struct fault_frame_s *sp, uint32_t typ, const void *fp) {
    char msg[200];
    const char *p;

    /* TODO: stop motors if possible */

start:
    sprintf(msg, "\r\n\nWe hit a %s at pc %p wirh lr %p\r\n"
        "Send 'r' to reboot openbgc, 'b' to attempt backtrace, any other key to repeat\r\n",
        typ == 0 ? "Hard Fault" : (typ == 1 ? "Mem Manage fault" :
            (typ == 2 ? "Bus Fault" : "Usage Fault")),
        sp->pc, sp->lr);
    /*
     * Use the PC value with gdb, e.g.:
     * $ gdb-multiarch .pio/build/simplebgc32_regular/firmware.elf
     * (gdb) info symbol 0x80067a8
     * loop + 156 in section .text
     */

    for (p = msg; *p; p++)
        crash_usart_write(USART1, *p);

    switch (crash_usart_read(USART1)) {
    case 'r':
        break;
    case 'R':
        /* Same as shutdown_to_bl(), see comments there */
        __disable_irq();
        USB->CNTR = 0x0003;
        HAL_RCC_DeInit();
        SysTick->CTRL = 0;
        SysTick->LOAD = 0;
        SysTick->VAL = 0;
        for (int i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
            NVIC->ICER[i] = 0xffffffff;
            NVIC->ICPR[i] = 0xffffffff;
        }
        __set_MSP(*(uint32_t *) 0x1fffd800);
        __set_PSP(*(uint32_t *) 0x1fffd800);
        ((void (*)(void)) *(uint32_t *) 0x1fffd804)();
        while (1);
    case 'b':
#ifdef __ARM_FP
        more_regs(sp);
        bt(sp, fp, false, 10);
        goto start;
#endif
    case 'B':
        more_regs(sp);
        bt(sp, fp, true, 20);
        goto start;
    default:
        goto start;
    }

    NVIC_SystemReset();
    while (1);
}
