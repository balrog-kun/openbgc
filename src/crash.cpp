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
        "b crash_handler \n"
        ::: "r0", "r1"
    );
    while (1);
}

HardwareSerial *error_serial; /* TODO: use the usart number from this */

static void crash_usart_write(USART_TypeDef *USARTx, uint8_t ch) {
    /* STM32F3-specific, F1/F4 would be slightly different */
    while (!(USARTx->ISR & USART_ISR_TXE));  /* Wait until TX buffer empty */
    USARTx->TDR = ch;                        /* Write byte */
}

static uint8_t crash_usart_read(USART_TypeDef *USARTx) {
    while (!(USARTx->ISR & USART_ISR_RXNE)); /* Wait until RX buffer not empty */
    return USARTx->RDR;
}

extern "C" void crash_handler(uint32_t *sp, uint32_t typ) {
    char msg[200];
    char *p;

    /* TODO: stop motors if possible */

start:
    sprintf(msg, "\r\n\nWe hit a %s at pc %p\r\nlr = %p, psr = 0x%08lx, r12 = 0x%08lx\r\n"
        "Send 'r' to reboot openbgc, any other key to repeat\r\n",
        typ == 0 ? "Hard Fault" : (typ == 1 ? "Mem Manage fault" :
            (typ == 2 ? "Bus Fault" : "Usage Fault")),
        sp[6], sp[5], sp[7], sp[4]);
    /*
     * Use the PC value with gdb, e.g.:
     * $ gdb-multiarch .pio/build/simplebgc32_regular/firmware.elf
     * (gdb) info symbol 0x80067a8
     * loop + 156 in section .text
     */

    for (p = msg; *p; p++)
        crash_usart_write(USART1, *p);

    if (crash_usart_read(USART1) != 'r')
        goto start;

    NVIC_SystemReset();
    while (1);
}
