#include "ctn91xx.h"
#include "ctn91xx_event.h"
#include "ctn91xx_util.h"
#include "ctn91xx_driver.h"
#include "ctn91xx_rpc.h"


void ctn91xx_event_cleanup_waiting(ctn91xx_dev_t* dev)
{
    struct list_head* i = NULL;
    ctn91xx_event_t* event = NULL;
    int not_done = 1;
    int spin_count = 0;
    spin_lock_w_flags(&dev->event_queue_lock);

    if(list_empty(&dev->event_queue)) {
        not_done = 0;
    }

    while(not_done) {
        list_for_each(i, &dev->event_queue) {
            event = list_entry(i, ctn91xx_event_t, list);

            if(event->list.next == &dev->event_queue) {
                not_done = 0;
            }

            //cleanup
            list_del(&event->list);
            ctn91xx_event_cleanup(dev, event);
            break;
        }
        spin_count++;
        if( spin_count > 100 ) {
            WARNING("event cleanup took too long, breaking out");
            break;
        }
    }
    spin_unlock_w_flags(&dev->event_queue_lock);
}


void ctn91xx_event_reset(ctn91xx_dev_t* dev)
{
    ctn91xx_event_cleanup_waiting(dev);
}

int ctn91xx_event_ready(ctn91xx_dev_t* dev)
{
    int ret = 0;
    spin_lock_w_flags(&dev->event_queue_lock);
    if(!list_empty(&dev->event_queue)) {
        ret = 1;
    }
    spin_unlock_w_flags(&dev->event_queue_lock);
    return ret;
}

ctn91xx_event_t* ctn91xx_remove_first_event(ctn91xx_dev_t* dev)
{
    ctn91xx_event_t* event = NULL;
    spin_lock_w_flags(&dev->event_queue_lock);
    if(!list_empty(&dev->event_queue)) {
        event = list_entry(dev->event_queue.next, ctn91xx_event_t, list);
        list_del(&event->list);
    }

    spin_unlock_w_flags(&dev->event_queue_lock);
    return event;
}

ctn91xx_event_t* ctn91xx_remove_first_delayed_event(ctn91xx_dev_t* dev)
{
    ctn91xx_event_t* event = NULL;
    spin_lock_w_flags(&dev->delayed_event_queue_lock);
    if(!list_empty(&dev->delayed_event_queue)) {
        event = list_entry(dev->delayed_event_queue.next, ctn91xx_event_t, list);
        list_del(&event->list);
    }

    spin_unlock_w_flags(&dev->delayed_event_queue_lock);
    return event;
}


void ctn91xx_event_cleanup(ctn91xx_dev_t* dev, ctn91xx_event_t* event)
{
    if(event) {
        kfree(event);
    }
}

unsigned int ctn91xx_event_poll(struct file* filp, struct poll_table_struct* wait)
{
    ctn91xx_dev_t* dev = NULL;
    unsigned int mask = 0;

    dev = ctn91xx_lookup_dev_from_file(NULL, filp);

    if(ctn91xx_event_ready(dev) && dev->event_user_cnt) {
        mask = POLLIN | POLLRDNORM;
    }

    poll_wait(filp, &dev->event_waitqueue, wait);

    return mask;
}

ssize_t ctn91xx_event_read_common(ctn91xx_dev_t* dev, char __user * data, size_t count, int nonblock)
{
    DECLARE_WAITQUEUE(wait, current);
    ssize_t retval = 0;
    ctn91xx_event_t* event = NULL;
    add_wait_queue(&dev->event_waitqueue, &wait);

    do {
        int should_break = 0;
restart_wait:
        should_break = 0;

        __set_current_state(TASK_INTERRUPTIBLE);


        should_break = ctn91xx_event_ready(dev);

        if(should_break)
            break;

        if(!dev->event_user_cnt) {
            retval = -EAGAIN;
            goto cleanup;
        }

        if(nonblock) {
            retval = -EAGAIN;
            goto cleanup;
        }

        if(signal_pending(current)) {
            retval = -ERESTARTSYS;
            INFO("event read interrupted");
            goto cleanup;
        }

        schedule();
    } while(1);

    if(count < SIZEOF_EVENT_HEADER) {
        ERROR("event read buffer too small, needs to be %d",
              SIZEOF_EVENT_HEADER);
        retval = -EAGAIN;
        goto cleanup;
    }

    event = ctn91xx_remove_first_event(dev);

    if(!event) {
        goto restart_wait;
    }

    if(copy_to_user((void __user*)data, &event->type, 1)) {
        retval = -EIO;
        goto cleanup;
    }
    if(copy_to_user((void __user*)data + 1, &event->length, 2)) {
        retval = -EIO;
        goto cleanup;
    }
    if(copy_to_user((void __user*)data + 3, &event->origin, 1)) {
        retval = -EIO;
        goto cleanup;
    }
    if(copy_to_user((void __user*)data + 4, &event->control_byte, 1)) {
        retval = -EIO;
        goto cleanup;
    }
    if(copy_to_user((void __user*)data + 5, &event->seqno, 1)) {
        retval = -EIO;
        goto cleanup;
    }

    retval = SIZEOF_EVENT_HEADER;

cleanup:
    if(event) {
        ctn91xx_event_cleanup(dev, event);
    }

    current->state = TASK_RUNNING;
    remove_wait_queue(&dev->event_waitqueue, &wait);

    return retval;
}

ssize_t ctn91xx_event_read(struct file* filp, char __user * data, size_t count, loff_t* offset)
{
    ctn91xx_dev_t* dev = ctn91xx_lookup_dev_from_file( NULL, filp );
    return ctn91xx_event_read_common( dev, data, count, filp->f_flags & O_NONBLOCK );
}

int ctn91xx_ioctl_event_read(ctn91xx_dev_t* dev, unsigned long arg, int compat)
{
    if( compat ) {
        event_read_compat_t read_data;
        if( copy_from_user( &read_data, (event_read_compat_t*) arg, sizeof(event_read_compat_t) ) ) {
            ERROR("input");
            return -EIO;
        }

        return ctn91xx_event_read_common( dev, (char __user*)(unsigned long)read_data.data, read_data.count, 0 );
    } else {
        event_read_t read_data;
        if( copy_from_user( &read_data, (event_read_t*) arg, sizeof(event_read_t) ) ) {
            ERROR("input");
            return -EIO;
        }

        return ctn91xx_event_read_common( dev, read_data.data, read_data.count, 0 );
    }
}

ctn91xx_event_t* ctn91xx_event_new(ctn91xx_dev_t* dev, cablecard_interrupt_t* data)
{
    return ctn91xx_event_new_full(dev, data->type, data->length, data->origin, data->control_byte, data->seqno);
}

ctn91xx_event_t* ctn91xx_event_new_full(
    ctn91xx_dev_t* dev,
    uint8_t type,
    uint16_t length,
    uint8_t origin,
    uint8_t control_byte,
    uint8_t seqno)
{
    ctn91xx_event_t* event = NULL;
    uint8_t should_queue = 1;

    if(!dev->event_user_cnt) {

            if( type == CTN91XX_EVENT_RPC_RECVD ) {
#if USE_LEON
                should_queue = 1;
#else
                return NULL;
#endif
            } else {
                return NULL;
            }
    }

    event = kmalloc(sizeof(ctn91xx_event_t), GFP_ATOMIC);
    if(!event) {
        ERROR("kmalloc failed. event dropped");
        return NULL;
    }

    memset(event, 0, sizeof(ctn91xx_event_t));
    event->type = type;
    event->length = length;
    event->origin = origin;
    event->control_byte = control_byte;
    event->seqno = seqno;
    event->should_queue = should_queue;

    return event;
}

void ctn91xx_queue_event(ctn91xx_dev_t* dev, ctn91xx_event_t* event)
{
    spin_lock_w_flags(&dev->event_queue_lock);
    list_add_tail(&event->list,&dev->event_queue);
    spin_unlock_w_flags(&dev->event_queue_lock);

    wake_up_interruptible(&dev->event_waitqueue);
}

void ctn91xx_queue_delayed_event(ctn91xx_dev_t* dev, ctn91xx_event_t* event)
{
    spin_lock_w_flags(&dev->delayed_event_queue_lock);
    list_add_tail(&event->list,&dev->delayed_event_queue);
    spin_unlock_w_flags(&dev->delayed_event_queue_lock);

    schedule_work(&dev->delayed_event_worker);
}


void ctn91xx_handle_delayed_event( ctn91xx_dev_t* dev, ctn91xx_event_t* event )
{
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
void ctn91xx_delayed_event_worker(void* work)
#else
void ctn91xx_delayed_event_worker(struct work_struct* work)
#endif
{
    ctn91xx_dev_t* dev = container_of(work, ctn91xx_dev_t, delayed_event_worker);
    ctn91xx_event_t* event = ctn91xx_remove_first_delayed_event(dev);

    ctn91xx_handle_delayed_event( dev, event );

    if(event->should_queue) {
        ctn91xx_queue_event(dev, event);
    } else {
        ctn91xx_event_cleanup(dev, event);
    }
}

#define CONTROL_MSG_SLOT_NUMBER 0x8

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
void ctn91xx_delayed_slot_number_worker( void* work )
#else
void ctn91xx_delayed_slot_number_worker( struct work_struct* work )
#endif
{
    rpc_send_t rpc;
    ctn91xx_dev_t* dev = container_of(work, ctn91xx_dev_t, delayed_slot_number_worker);

    INFO("sending slot number %d", dev->board_number);

    rpc.msg = kmalloc( 8, GFP_ATOMIC );
    rpc.length = 8;
    rpc.section_length = 8;
    rpc.payload_start = 1;

    rpc.msg[0] = CONTROL_MSG_SLOT_NUMBER;
    *((uint32_t*)&rpc.msg[4]) = htonl( (dev->face_present << 8) | dev->board_number );
    ctn91xx_rpc_send_kernel( dev, &rpc );
    kfree( rpc.msg );
}

void ctn91xx_event_dump_queue(ctn91xx_dev_t* dev)
{
    struct list_head* i = NULL;
    ctn91xx_event_t* event = NULL;
    spin_lock_w_flags(&dev->event_queue_lock);

    if(list_empty(&dev->event_queue)) {
        goto end;
    }

    list_for_each(i, &dev->event_queue) {
        event = list_entry(i, ctn91xx_event_t, list);
        INFO("\tevent %p, type %#02x", event, event->type);
    }

end:
    spin_unlock_w_flags(&dev->event_queue_lock);
}

