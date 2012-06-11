#ifndef CTN91XX_INTERRUPT_H
#define CTN91XX_INTERRUPT_H

#include "ctn91xx.h"

void cablecard_interrupt_handle(cablecard_interrupt_t* data, ctn91xx_dev_t* dev);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
irqreturn_t ctn91xx_isr(int irq, void *ptr, struct pt_regs* regs);
#else
irqreturn_t ctn91xx_isr(int irq, void *ptr);
#endif
irqreturn_t cablecard_reg_isr(int irq, void *ptr);
irqreturn_t ctn91xx_timer_isr(int irq, void *ptr);
irqreturn_t i2c_isr(int irq, void *ptr);
irqreturn_t spit_isr(int irq, void *ptr);
irqreturn_t ctn91xx_mpeg_isr(int irq, void *ptr);
irqreturn_t drm_reg_isr(int irq, void *ptr);
irqreturn_t gpio_isr(int irq, void* ptr);
irqreturn_t scte21_isr(int irq, void* ptr);

#endif
