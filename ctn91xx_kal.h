#ifndef _CTN91XX_KAL_H
#define _CTN91XX_KAL_H

#if defined(__KERNEL__)
#include <linux/types.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/sched.h>
#include <linux/pagemap.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/poll.h>
#include <linux/time.h>
#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/kdev_t.h>
#include <linux/file.h>
#include <linux/vmalloc.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/cdev.h>
#include <linux/list.h>

//If kernel version >= 4.11, require <linux/sched/signal.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(4,11,0)
    #include <linux/sched/signal.h>
#endif

//it's in an arch specific header...this is just easier
#define TCGETS 0x5401


#define writell(_d,_p) (*(volatile uint64_t*)(_p)) = (uint64_t)_d
#define readll(_p) ((volatile uint64_t*)(_p))[0]

#define spin_lock_w_flags(_l) unsigned long _flags;  spin_lock_irqsave((_l), _flags)
#define spin_relock_w_flags(_l) spin_lock_irqsave((_l), _flags)
#define spin_unlock_w_flags(_l) spin_unlock_irqrestore((_l), _flags)

#endif

#endif

