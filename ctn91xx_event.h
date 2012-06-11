#ifndef CTN91XX_SIGNAL_H
#define CTN91XX_SIGNAL_H

#include "ctn91xx.h"

ctn91xx_event_t* ctn91xx_event_new(ctn91xx_dev_t* dev, cablecard_interrupt_t* data);

ctn91xx_event_t* ctn91xx_event_new_full(
    ctn91xx_dev_t* dev,
    uint8_t type,
    uint16_t length,
    uint8_t origin,
    uint8_t control_byte,
    uint8_t seqno);

ssize_t ctn91xx_event_read(struct file* filp, char __user * data, size_t count, loff_t* offset);
unsigned int ctn91xx_event_poll(struct file* filp, struct poll_table_struct* wait);
ctn91xx_event_t* ctn91xx_remove_first_event(ctn91xx_dev_t* dev);
void ctn91xx_event_cleanup(ctn91xx_dev_t* dev, ctn91xx_event_t* event);
void ctn91xx_event_cleanup_waiting(ctn91xx_dev_t* dev);

void ctn91xx_queue_event(ctn91xx_dev_t* dev, ctn91xx_event_t* event);
void ctn91xx_queue_delayed_event(ctn91xx_dev_t* dev, ctn91xx_event_t* event);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
void ctn91xx_delayed_event_worker(void* work);
void ctn91xx_delayed_slot_number_worker( void* work );
#else
void ctn91xx_delayed_event_worker(struct work_struct* work);
void ctn91xx_delayed_slot_number_worker( struct work_struct* work );
#endif


void ctn91xx_event_reset(ctn91xx_dev_t* dev);

void ctn91xx_event_dump_queue(ctn91xx_dev_t* dev);

int ctn91xx_ioctl_event_read(ctn91xx_dev_t* dev, unsigned long arg, int compat);

#endif
