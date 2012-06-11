#include "ctn91xx.h"
#include "ctn91xx_util.h"
#include "ctn91xx_driver.h"
#include "ctn91xx_mpeg.h"
#include "ctn91xx_reset.h"

#include <linux/dma-mapping.h>

#define MPEG_VERIFY 0

#if CHECK_NOTIFY_CC
#define CHECK_NOTIFY_CC_ACCUM_AMOUNT 100
#define CHECK_NOTIFY_SYNC_ACCUM_AMOUNT 1000

void notify_cc_check( ctn91xx_dev_t* dev, int tuner_index, uint8_t* buffer, uint32_t length )
{
    int pkts = length/188;
    int i = 0;
    for (i = 0; i < pkts; i++)
    {
        uint8_t* pkt = buffer + (188 * i);
        uint16_t pid = ((pkt[1] & 0x1f) << 8) | (pkt[2] & 0xff);
        uint8_t continuity_counter;
        uint16_t expected_cc;
        uint8_t has_adaptation = (((pkt[3] >> 5) & 0x1));
        uint8_t adaptation_length = pkt[4];
        uint8_t adaptation_flags = pkt[5];

        if( pid >= 0x1fff ) {
            continue;
        }

        if( pkt[0] != 0x47 ) {
            dev->pid_cc_notify[tuner_index][pid].out_of_sync++;
            if( dev->pid_cc_notify[tuner_index][pid].out_of_sync % CHECK_NOTIFY_SYNC_ACCUM_AMOUNT == 0 ) {
                if( printk_ratelimit() ) {
                    WARNING("NOTIFY board %d tuner %d pid %#x out_of_sync %d",
                            dev->board_number,
                            tuner_index,
                            pid,
                            dev->pid_cc_notify[tuner_index][pid].out_of_sync);
                }
            }
            continue;
        }

        continuity_counter = (pkt[3] & 0x0f);

        expected_cc = (dev->pid_cc_notify[tuner_index][pid].last_cc + 1) & 0xf;
        if (expected_cc != continuity_counter
                && continuity_counter != dev->pid_cc_notify[tuner_index][pid].last_cc
                && (!has_adaptation || adaptation_length == 0 || !(adaptation_flags & 0x80))) //discontinuity_indicator
        {
            dev->pid_cc_notify[tuner_index][pid].discontinuities++;
            dev->stats.discont_count[tuner_index]++;

            /*
            if(dev->pid_cc_notify[tuner_index][pid].discontinuities % CHECK_NOTIFY_CC_ACCUM_AMOUNT == 0 || dev->pid_cc_notify[tuner_index][pid].discontinuities == 2) {
                if( printk_ratelimit() ) {
                    WARNING( "NOTIFY board %d tuner %d pid %#x discontinuities %d count %d expected_cc %#01x cc %#01x",
                            dev->board_number,
                            tuner_index,
                            pid,
                            dev->pid_cc_notify[tuner_index][pid].discontinuities,
                            dev->pid_cc_notify[tuner_index][pid].count,
                            expected_cc,
                            continuity_counter);
                }
            }*/
        }
        dev->pid_cc_notify[tuner_index][pid].count++;
        dev->pid_cc_notify[tuner_index][pid].last_cc = continuity_counter;
    }
}
#endif

void ctn91xx_reinit_mpeg_registers( ctn91xx_dev_t* dev )
{
#if HAS_MPEG_DMA
    mpeg_reset( dev );
#endif
}

void ctn91xx_vbuffer_print_internal(ctn91xx_dev_t* dev, vbuffer_t* vbuffer);


void verify_buffer_state(ctn91xx_dev_t* dev, vbuffer_t* vbuffer, int who_called )
{
#if !MPEG_VERIFY
    return;
#else
    uint32_t j;
    uint32_t num_bank_pages = default_num_pages_per_bank( vbuffer->tuner_index );
    static uint32_t cnt=0;

    uint32_t notify_abs = ( vbuffer->notify_idx < vbuffer->write_idx ? vbuffer->npages + vbuffer->notify_idx : vbuffer->notify_idx );

    static uint32_t printed = 0;
    if(printed) return;
    cnt++;

    if( !( notify_abs >= vbuffer->write_idx
            && ( notify_abs <= ( vbuffer->write_idx + num_bank_pages ) ) ) ) {
        printed = 1;
        WARNING("verify1 failed (%d, %d): notify_index=%d notify_abs=%d write_index=%d num_bank_pages=%d", who_called, cnt, vbuffer->notify_idx, notify_abs, vbuffer->write_idx, num_bank_pages);
    }

    for (j = 0; j < vbuffer->npages; j++) {

        uint32_t j_abs = ( j < vbuffer->write_idx ? vbuffer->npages + j : j);
        if(vbuffer->lock_cnt[j]) {
            if(!(j_abs >= notify_abs && j_abs < vbuffer->write_idx + 2*num_bank_pages)) {
                printed = 1;
                WARNING("verify3 failed (%d, %d): j=%d j_abs=%d notify_index=%d notify_abs=%d write_index=%d", who_called, cnt, j, j_abs, vbuffer->notify_idx, notify_abs, vbuffer->write_idx);
            }
        }

        if(vbuffer->dropped[j]) {
            if(!(j_abs >= vbuffer->write_idx && j_abs < vbuffer->write_idx + num_bank_pages)) {
                printed = 1;
                WARNING("verify4 failed (%d, %d): j=%d j_abs=%d notify_index=%d notify_abs=%d write_index=%d", who_called, cnt, j, j_abs, vbuffer->notify_idx, notify_abs, vbuffer->write_idx);
            }
        }
    }

    if( printed ) {
        ctn91xx_vbuffer_print_internal( dev, vbuffer );
    }
#endif
}


#if HAS_MPEG_DMA

int is_filter_stream(int stream_index) {
    return (stream_index + CTN91XX_ORIGIN_TUNER0 >= CTN91XX_ORIGIN_FILTER0);
}

int pages_per_mpeg_device(int stream_index) {
    switch(stream_index) {
        case (0)...(5):
            return MPEG_BUFFER_NPAGES;
        case (6)...(11):
        default:
            return FILTER_BUFFER_NPAGES;
    }
}

int num_pages_per_bank(ctn91xx_dev_t* dev, int stream_index) {

    return default_num_pages_per_bank(stream_index);
}

int num_pages_per_notify(ctn91xx_dev_t* dev, int stream_index) {

    if(is_filter_stream(stream_index)) {
        return ctn91xx_read8(dev->mpeg_filter_dma_base,
                             MPEG_FILTER_DMA_PAGES_PER_NOTIFY_BASE + ((stream_index+CTN91XX_ORIGIN_TUNER0-CTN91XX_ORIGIN_FILTER0)*8));
    } else {
        return ctn91xx_read8(dev->mpeg_dma_base, MPEG_DMA_PAGES_PER_NOTIFY_BASE + (stream_index*8));
    }
}

int mpeg_bytes_per_page_from_stream(ctn91xx_dev_t* dev, int stream_index) {

    if(is_filter_stream(stream_index)) {
        return ctn91xx_read16(dev->mpeg_filter_dma_base,
                              MPEG_FILTER_DMA_BYTES_PER_PAGE_BASE + ((stream_index+CTN91XX_ORIGIN_TUNER0-CTN91XX_ORIGIN_FILTER0)*2));
    } else {
        return ctn91xx_read16(dev->mpeg_dma_base, MPEG_DMA_BYTES_PER_PAGE_BASE + (stream_index*2));
    }
}

int default_num_pages_per_bank(int stream_index) {
    switch(stream_index) {
        case (0)...(5):
#if USE_MPEG_NOTIFY
#if USE_LEON
            return 2;
#else
            return 32;
#endif
#else
            return 8;
#endif
        case (6)...(11):
#if USE_MPEG_NOTIFY
#if USE_LEON
            return 4;
#else
            return 8;
#endif
#else
            return 1;
#endif
        default:
            return 0;
    }
}

int default_num_pages_per_notify(int stream_index) {
    switch(stream_index) {
        case (0)...(5):
#if USE_LEON
            return 2;
#else
            return 8;
#endif
        case (6)...(11):
#if USE_MPEG_NOTIFY && USE_LEON
            return 1;
#else
            return 1;
#endif
        default:
            return 0;
    }
}

int default_mpeg_bytes_per_page_from_stream(int stream_index)
{
    switch(stream_index) {
        case (0)...(5):
            return 3948;
        case (6)...(11):
            return 188;
        default:
            return 0;
    }
}

int get_addr_list_offset(ctn91xx_dev_t* dev, int stream_index, int bank)
{
    int ret = 0;
    switch(stream_index) {
        case (0)...(5):
            ret = 128 + (stream_index * 128 * 4);
            if(bank == 1) {
                ret += 64*4;
            }
            break;
        case (6)...(11):
            ret = 512 + 64*(stream_index + CTN91XX_ORIGIN_TUNER0 - CTN91XX_ORIGIN_FILTER0);
            if(bank == 1) {
                ret += 8*4;
            }
            break;
    }
    return ret;
}

void mpeg_vbuffer_from_tuner_index(uint8_t tuner_index, ctn91xx_dev_t* dev,
                             vbuffer_t** vbuffer)
{
    if( !vbuffer ) {
        ERROR("invalid arg");
        return;
    }

    if( tuner_index > NUM_MPEG_DEVICES ) {
        *vbuffer = NULL;
        return;
    }

    *vbuffer = &dev->mpeg_buffer[tuner_index];
}


void mpeg_vbuffer_from_tuner(uint8_t tuner, ctn91xx_dev_t* dev,
                             vbuffer_t** vbuffer)
{
    if( !vbuffer ) {
        ERROR("invalid arg");
        return;
    }

    if( tuner > CTN91XX_ORIGIN_TUNER0+CTN91XX_ORIGIN_FILTER5) {
        *vbuffer = NULL;
        return;
    }

    *vbuffer = &dev->mpeg_buffer[tuner - CTN91XX_ORIGIN_TUNER0];
}

void mpeg_user_cnt(uint8_t minor_num, ctn91xx_dev_t* dev, int** user_cnt, uint8_t* origin)
{
    uint8_t device_number = CETON_MINOR_DEVICE_NUMBER( minor_num );
    int * local_user_cnt = 0;
    uint8_t local_origin = 0;

    if (!dev || (!user_cnt && !origin)) {
        ERROR("invalid arg");
        return;
    }

    if( device_number < 1 || device_number > ( NUM_MPEG_DEVICES + 1 ) ) {
        ERROR("invalid arg");
        return;
    }

    device_number--;
    local_origin = device_number + CTN91XX_ORIGIN_TUNER0;
    local_user_cnt = &dev->mpeg_user_cnt[device_number];

    if (origin) {
        *origin = local_origin;
    }
    if (user_cnt) {
        *user_cnt = local_user_cnt;
    }
}


void mpeg_vbuffer_from_minor(uint8_t minor_num, ctn91xx_dev_t* dev,
                             vbuffer_t** vbuffer)
{
    uint8_t device_number = CETON_MINOR_DEVICE_NUMBER( minor_num );

    if (!dev || !vbuffer) {
        ERROR("invalid arg");
        return;
    }

    if( device_number < 1 || device_number > ( NUM_MPEG_DEVICES + 1 ) ) {
        ERROR("invalid arg");
        return;
    }

    device_number--;

    *vbuffer = &dev->mpeg_buffer[device_number];
}

int* mpeg_user_cnt_from_tuner(ctn91xx_dev_t* dev, uint8_t origin)
{
    int* ret = NULL;

    if( origin > CTN91XX_ORIGIN_FILTER5 ) {
        ERROR("invalid arg");
        return NULL;
    }

    ret = &dev->mpeg_user_cnt[origin - CTN91XX_ORIGIN_TUNER0];

    if( ret && ( *ret > 1 || *ret < 0 ) ) {
        ERROR("mpeg user cnt invalid tuner: %d cnt %d", origin, *ret);
    }

    return ret;
}

int ctn91xx_mpeg_dma_stats( ctn91xx_dev_t* dev, unsigned long arg )
{
    vbuffer_t* vbuffer = NULL;
    dma_stats_t stats;
    if( copy_from_user( &stats, (dma_stats_t*)arg, sizeof( dma_stats_t ) ) ) {
        return -EIO;
    }

    mpeg_vbuffer_from_tuner( stats.tuner, dev, &vbuffer );

    if( !vbuffer ) {
        return -EINVAL;
    }

    stats.packet_count = vbuffer->packet_count;

    if( copy_to_user( (dma_stats_t*)arg, &stats, sizeof( dma_stats_t ) ) ) {
        return -EIO;
    }
    return 0;
}

static int ctn91xx_mpeg_open(struct inode* inode, struct file* filp)
{
    ctn91xx_dev_t* dev = NULL;
    int ret = 0;
    uint8_t tuner;
    int* user_cnt = 0;
    int minor;
    vbuffer_t* vbuffer = NULL;

    dev = ctn91xx_lookup_dev_from_file(inode, filp);
    minor = iminor(inode);

    mpeg_user_cnt(minor, dev, &user_cnt, &tuner);

    if(!user_cnt) {
        ctn91xx_cleanup_dev_from_file(inode, filp);
        return -EIO;
    }

    mutex_lock(&dev->fd_mutex);
    if(!(*user_cnt)) {
        (*user_cnt)++;
    } else {
        ctn91xx_cleanup_dev_from_file(inode, filp);
        ret = -EBUSY;
        goto cleanup;
    }

    mpeg_vbuffer_from_minor((uint8_t)minor, dev, &vbuffer);

    {
        spin_lock_w_flags(&vbuffer->lock);
        vbuffer->read_idx = vbuffer->notify_idx;
        spin_unlock_w_flags(&vbuffer->lock);
    }

cleanup:
    mutex_unlock(&dev->fd_mutex);
    return ret;
}

static int ctn91xx_mpeg_release(struct inode* inode, struct file* filp)
{
    ctn91xx_dev_t* dev = NULL;
    uint8_t tuner;
    int* user_cnt;
    int minor;


    dev = ctn91xx_lookup_dev_from_file(inode, filp);
    minor = iminor(inode);

    mpeg_user_cnt(iminor(inode), dev, &user_cnt, &tuner);

    if(!user_cnt) {
        ctn91xx_cleanup_dev_from_file(inode, filp);
        return -EIO;
    }

    mutex_lock(&dev->fd_mutex);
    if((*user_cnt)) {
        (*user_cnt)--;
    }
    mutex_unlock(&dev->fd_mutex);
    ctn91xx_cleanup_dev_from_file(inode, filp);
    return 0;
}

static int ctn91xx_mpeg_mmap(struct file* filp, struct vm_area_struct* vma)
{
    int ret = 0;
    ctn91xx_dev_t* dev = NULL;
    int minor;
    vbuffer_t* vbuffer = NULL;
    long length = vma->vm_end - vma->vm_start;
    unsigned long start = vma->vm_start;
    int page_idx = 0;

    dev = ctn91xx_lookup_dev_from_file(NULL, filp);
    minor = ctn91xx_lookup_minor_from_file(NULL, filp);

    mpeg_vbuffer_from_minor((uint8_t)minor, dev, &vbuffer);

    if( !vbuffer ) {
        return -EINVAL;
    }

    if ( ( vma->vm_end - vma->vm_start ) > ( vbuffer->npages * PAGE_SIZE ) ) {
        ERROR( "requested region size is too big" );
        return -1;
    }

    while( length > 0 ) {
        struct page* p = vbuffer->pages[page_idx++];
        if( ( ret = vm_insert_page( vma, start, p ) ) < 0 ) {
            return ret;
        }
        start += PAGE_SIZE;
        length -= PAGE_SIZE;
    }

    return ret;
}


//vbuffer->lock needs to be locked when calling this
int mpeg_data_ready(vbuffer_t* vbuffer)
{
    return (vbuffer->lock_cnt[vbuffer->read_idx] == 0) && (vbuffer->remaining[vbuffer->read_idx]);
}

int mpeg_data_ready_at_index( vbuffer_t* vbuffer, int idx )
{
    return ( vbuffer->lock_cnt[idx] == 0 ) && ( vbuffer->remaining[idx] );
}

#define ACCUM_AMOUNT 100

static unsigned int ctn91xx_mpeg_poll(struct file* filp, struct poll_table_struct* wait)
{
    ctn91xx_dev_t* dev = NULL;
    int minor;
    uint8_t tuner;
    int tuner_index;
    int* user_cnt;
    uint32_t mask = 0;
    vbuffer_t* vbuffer = 0;

    dev = ctn91xx_lookup_dev_from_file(NULL, filp);
    minor = ctn91xx_lookup_minor_from_file(NULL, filp);

    mpeg_user_cnt((uint8_t)minor, dev, &user_cnt, &tuner);

    mpeg_vbuffer_from_minor((uint8_t)minor, dev, &vbuffer);

    tuner_index = tuner - CTN91XX_ORIGIN_TUNER0;

    if(vbuffer) {
        spin_lock_w_flags(&vbuffer->lock);

        if(mpeg_data_ready(vbuffer) && *user_cnt) {
            mask = POLLIN | POLLRDNORM;
        }

        spin_unlock_w_flags(&vbuffer->lock);

    } else {
        printk("vbuffer == NULL\n");
    }

    //Note: this debug stuff is not synced
    if(((dev->stats.drop_count[tuner_index] - 1) % (ACCUM_AMOUNT/10)) == 0) {
        if( printk_ratelimit() ) {
            printk("(in poll): board %d drop_count %d tuner_index %d mask %d\n",
                   dev->board_number, dev->stats.drop_count[tuner_index], tuner_index, mask);
        }
        dev->stats.drop_count[tuner_index]++;
    }

    poll_wait(filp, &dev->mpeg_wait_queues[tuner_index], wait);

    return mask;
}

ssize_t ctn91xx_mpeg_read_common( ctn91xx_dev_t* dev, vbuffer_t* vbuffer, char __user * data, size_t count, int tuner_index, int nonblocking );

ssize_t ctn91xx_mpeg_read(struct file* filp, char __user * data, size_t count, loff_t* offset)
{
    int minor;
    ctn91xx_dev_t* dev = NULL;
    vbuffer_t* vbuffer = NULL;
    int* user_cnt;
    uint8_t tuner;
    int tuner_index;

    dev = ctn91xx_lookup_dev_from_file(NULL, filp);
    minor = ctn91xx_lookup_minor_from_file(NULL, filp);

    mpeg_user_cnt((uint8_t)minor, dev, &user_cnt, &tuner);

    mpeg_vbuffer_from_minor((uint8_t)minor, dev, &vbuffer);

    tuner_index = tuner - CTN91XX_ORIGIN_TUNER0;

    return ctn91xx_mpeg_read_common( dev, vbuffer, data, count, tuner_index, filp->f_flags & O_NONBLOCK );
}


ssize_t ctn91xx_mpeg_read_common( ctn91xx_dev_t* dev, vbuffer_t* vbuffer, char __user * data,
                                  size_t count, int tuner_index, int nonblocking)
{
    DECLARE_WAITQUEUE(wait, current);
    ssize_t retval = 0;
    int done = 0;
    int error = 0;
    size_t saved_cnt = count;
    spinlock_t* lock = 0;

    if(!vbuffer) {
        ERROR("bad vbuffer");
        return -EIO;
    }

    //Note: this debug stuff is not synced
    if(((dev->stats.drop_count[tuner_index] - 1) % (ACCUM_AMOUNT/10)) == 0) {
        if( printk_ratelimit() ) {
            printk("(in read): drop_count %d tuner_index %d\n",
                   dev->stats.drop_count[tuner_index], tuner_index);
        }
        dev->stats.drop_count[tuner_index]++;
    }

    lock = &vbuffer->lock;

    add_wait_queue(&dev->mpeg_wait_queues[tuner_index], &wait);

    do {
        int should_break = 0;
restart_wait:
        should_break = 0;

        set_current_state(TASK_INTERRUPTIBLE);

        //Check for data
        {
            spin_lock_w_flags(lock);

            if(mpeg_data_ready(vbuffer)) {
                should_break = 1;
            }

            spin_unlock_w_flags(lock);
        }

        if(should_break) {
            break;
        }

        if(nonblocking) {
            retval = -EAGAIN;
            goto out;
        }

        if(!dev->mpeg_user_cnt[vbuffer->tuner_index]) {
            retval = -EAGAIN;
            goto out;
        }

        if(signal_pending(current)) {
            retval = -ERESTARTSYS;
            goto out;
        }

        schedule();
    } while(1);

    {
        spin_lock_w_flags(lock);

        while(!done) {

            if(!mpeg_data_ready(vbuffer) || !dev->mpeg_user_cnt[vbuffer->tuner_index]) {
                //go back and wait for the rest of my data
                spin_unlock_w_flags(lock);

                if (nonblocking) {
                    //We already used our data, none is left so just let them know how much they got
                    goto out;
                } else {
                    goto restart_wait;
                }
            }

            if(vbuffer->remaining[vbuffer->read_idx] >= count) {
                size_t offset = vbuffer->sizes[vbuffer->read_idx] - vbuffer->remaining[vbuffer->read_idx];

                spin_unlock_w_flags(lock);

                if(copy_to_user(&(data[saved_cnt - count]),
                                &(vbuffer->buffers[vbuffer->read_idx][offset]), count)) {
                    error = 1;
                }

                spin_relock_w_flags(lock);

                vbuffer->remaining[vbuffer->read_idx] -= count;
                retval += count;
                done = 1;

            } else if(vbuffer->remaining[vbuffer->read_idx] > 0) { //remaining < count
                size_t offset = vbuffer->sizes[vbuffer->read_idx] - vbuffer->remaining[vbuffer->read_idx];

                spin_unlock_w_flags(lock);

                if(copy_to_user(&(data[saved_cnt - count]),
                                &(vbuffer->buffers[vbuffer->read_idx][offset]), vbuffer->remaining[vbuffer->read_idx])) {
                    error = 1;
                    done = 1;
                }

                spin_relock_w_flags(lock);

                retval += vbuffer->remaining[vbuffer->read_idx];
                count -= vbuffer->remaining[vbuffer->read_idx];
                vbuffer->remaining[vbuffer->read_idx] = 0;
            } else {
                ERROR("mpeg read, bad state");
                error = 1;
                spin_unlock_w_flags(lock);

                goto out;
            }

            if(!vbuffer->remaining[vbuffer->read_idx]) {
                vbuffer->read_idx = WRAPPED_PAGE_INDEX( vbuffer, vbuffer->read_idx + 1 );
            }
        }

        spin_unlock_w_flags(lock);
    }

out:
    set_current_state(TASK_RUNNING);
    remove_wait_queue(&dev->mpeg_wait_queues[tuner_index], &wait);

    if(error) {
        retval = -EIO;
    }

    return retval;
}

int ctn91xx_ioctl_mpeg_read_setup(ctn91xx_dev_t* dev, int minor, unsigned long arg)
{
    vbuffer_t* vbuffer = NULL;
    mpeg_stream_setup_t setup;

    mpeg_vbuffer_from_minor((uint8_t)minor, dev, &vbuffer);

    if( !vbuffer ) {
        ERROR("invalid vbuffer");
        return -EINVAL;
    }

    {
        spin_lock_w_flags( &vbuffer->lock );
        setup.read_index = vbuffer->read_idx;
        spin_unlock_w_flags( &vbuffer->lock );
    }

    setup.npages = vbuffer->npages;
    setup.bytes_per_page = default_mpeg_bytes_per_page_from_stream( vbuffer->tuner_index );

    if( copy_to_user( (uint8_t*)arg, &setup, sizeof( mpeg_stream_setup_t ) ) ) {
        ERROR("invalid user ptr");
        return -EIO;
    }

    return 0;
}

int ctn91xx_ioctl_mpeg_read_complete(ctn91xx_dev_t* dev, int minor, unsigned long arg);

int ctn91xx_ioctl_mpeg_read_start(ctn91xx_dev_t* dev, int minor, unsigned long arg)
{
    ssize_t retval = 0;
    int done = 0;
    int error = 0;
    vbuffer_t* vbuffer = NULL;
    mpeg_read_t mpeg_data;
    size_t count;
    spinlock_t* lock = 0;

    if( copy_from_user( &mpeg_data, (mpeg_read_t*) arg, sizeof(mpeg_read_t) ) ) {
        return -EINVAL;
    }

    if( mpeg_data.amount_last_read ) {
        ERROR("unless USE_AMOUNT_LAST_READ is set, you shouldn't get here");
        if( ctn91xx_ioctl_mpeg_read_complete( dev, minor, arg ) < 0 ) {
            return -EINVAL;
        }
    }

    count = mpeg_data.count;

    mpeg_vbuffer_from_minor((uint8_t)minor, dev, &vbuffer);

    if( !vbuffer ) {
        return -EINVAL;
    }

    lock = &vbuffer->lock;

    {
        int temp_read_index;
        spin_lock_w_flags( lock );
        temp_read_index = vbuffer->read_idx;

        while( !done ) {
            int cur_remaining = vbuffer->remaining[temp_read_index];

            if( !mpeg_data_ready_at_index( vbuffer, temp_read_index ) ) {
                spin_unlock_w_flags( lock );
                goto out;
            }

            if( cur_remaining >= count ) {
                cur_remaining -= count;
                retval += count;
                done = 1;
            } else if( cur_remaining > 0 ) { //remaining < count
                retval += cur_remaining;
                count -= cur_remaining;
                cur_remaining = 0;
            } else {
                ERROR("mpeg read, bad state retval %zd count %zd cur_remaining %d",
                      retval, count, cur_remaining);
                error = 1;
                spin_unlock_w_flags(lock);
                goto out;
            }

            if( !cur_remaining ) {
                temp_read_index = WRAPPED_PAGE_INDEX(vbuffer, temp_read_index + 1 );
            }
        }

        spin_unlock_w_flags( lock );
    }

out:
    if( error ) {
        retval = -EIO;
    }

    return retval;
}



int ctn91xx_ioctl_mpeg_read_complete(ctn91xx_dev_t* dev, int minor, unsigned long arg)
{
    int done = 0;
    int error = 0;
    size_t count;
    int new_read_index = 0;
    vbuffer_t* vbuffer = NULL;
    mpeg_read_t mpeg_data;
    spinlock_t* lock = 0;

    if( copy_from_user( &mpeg_data, (mpeg_read_t*) arg, sizeof(mpeg_read_t) ) ) {
        return -EINVAL;
    }

    count = mpeg_data.amount_last_read;

    mpeg_vbuffer_from_minor((uint8_t)minor, dev, &vbuffer);

    if( !vbuffer ) {
        return -EINVAL;
    }

    new_read_index = vbuffer->read_idx;
    lock = &vbuffer->lock;

    {
        spin_lock_w_flags( lock );

        while( !done ) {

            if( !mpeg_data_ready( vbuffer ) || !dev->mpeg_user_cnt[vbuffer->tuner_index]) {
                spin_unlock_w_flags( lock );
                goto out;
            }

            if( vbuffer->remaining[vbuffer->read_idx] >= count ) {
                vbuffer->remaining[vbuffer->read_idx] -= count;
                done = 1;
            } else if( vbuffer->remaining[vbuffer->read_idx] > 0 ) { //remaining < count
                count -= vbuffer->remaining[vbuffer->read_idx];
                vbuffer->remaining[vbuffer->read_idx] = 0;
            } else {
                ERROR("mpeg read, bad state");
                error = 1;
                spin_unlock_w_flags( lock );
                goto out;
            }

            if( !vbuffer->remaining[vbuffer->read_idx] ) {
                vbuffer->read_idx = new_read_index = WRAPPED_PAGE_INDEX( vbuffer, vbuffer->read_idx + 1 );
            }
        }
        spin_unlock_w_flags( lock );
    }

out:
    if( error ) {
        return -EIO;
    }

    return new_read_index;
}

int ctn91xx_mpeg_send(ctn91xx_dev_t* dev, int minor, unsigned long arg)
{
    int ret = 0;
    int done = 0;
    vbuffer_t* vbuffer = NULL;
    mpeg_send_t send;
    struct file* out_file = NULL;
    spinlock_t* lock = 0;
    ssize_t written = 0;
    size_t count;

    mpeg_vbuffer_from_minor((uint8_t)minor, dev, &vbuffer);

    if( !vbuffer ) {
        return -EINVAL;
    }

    if( copy_from_user( &send, (mpeg_send_t*) arg, sizeof(mpeg_send_t) ) ) {
        return -EINVAL;
    }

    count = send.count;

    if( copy_from_user( vbuffer->header_buffer, send.header, send.header_length ) ) {
        ret =-EINVAL;
        goto out;
    }

    out_file = fget( send.out_fd );
    if( !out_file ) {
        goto out;
    }

    if( !out_file->f_op || !out_file->f_op->sendpage ) {
        goto fput_out;
    }

    written = out_file->f_op->sendpage(
                  out_file,
                  vbuffer->header_buffer_page,
                  0, send.header_length,
                  &out_file->f_pos, 1 /* more */ );

    lock = &vbuffer->lock;
    {
        spin_lock_w_flags( lock );

        while( !done ) {
            size_t offset = vbuffer->sizes[vbuffer->read_idx] - vbuffer->remaining[vbuffer->read_idx];

            if( !mpeg_data_ready( vbuffer ) || !dev->mpeg_user_cnt[vbuffer->tuner_index]) {
                spin_unlock_w_flags( lock );
                out_file->f_op->sendpage(
                    out_file,
                    NULL,
                    0,
                    0,
                    &out_file->f_pos, 0 /* no-more */ );
                goto fput_out;
            }

            if( vbuffer->remaining[vbuffer->read_idx] >= count ) {

                spin_unlock_w_flags( lock );
                written = out_file->f_op->sendpage(
                              out_file,
                              vbuffer->pages[vbuffer->read_idx],
                              offset,
                              count,
                              &out_file->f_pos, 0 /* no-more */ );

                if( written < 0 ) {
                    ERROR("sendpage, written %zd", written);
                    goto fput_out;
                }

                spin_relock_w_flags( lock );
                vbuffer->remaining[vbuffer->read_idx] -= count;
                ret += count;
                done = 1;

            } else if( vbuffer->remaining[vbuffer->read_idx] > 0 ) { //remaining < count

                spin_unlock_w_flags( lock );
                written = out_file->f_op->sendpage(
                              out_file,
                              vbuffer->pages[vbuffer->read_idx],
                              offset,
                              vbuffer->remaining[vbuffer->read_idx],
                              &out_file->f_pos, 1 /* more */ );

                if( written < 0 ) {
                    ERROR("sendpage, written %zd", written);
                    goto fput_out;
                }

                spin_relock_w_flags( lock );

                ret += vbuffer->remaining[vbuffer->read_idx];
                count -= vbuffer->remaining[vbuffer->read_idx];
                vbuffer->remaining[vbuffer->read_idx] = 0;
            } else {
                ERROR("mpeg send, bad state");
                ret = -EINVAL;
                spin_unlock_w_flags( lock );
                goto fput_out;
            }

            if( !vbuffer->remaining[vbuffer->read_idx] ) {
                vbuffer->read_idx = WRAPPED_PAGE_INDEX( vbuffer, vbuffer->read_idx + 1 );
            }
        }

        spin_unlock_w_flags( lock );
    }

fput_out:
    fput( out_file );
out:
    return ret;
}

ssize_t ctn91xx_mpeg_kernel_read( vbuffer_t* vbuffer, char* data, size_t count )
{
    ctn91xx_dev_t* dev = vbuffer->dev;
    ssize_t retval = 0;
    int done = 0;
    size_t saved_cnt = count;

    while(!done) {
        size_t offset = vbuffer->sizes[vbuffer->read_idx] - vbuffer->remaining[vbuffer->read_idx];

        if(!mpeg_data_ready(vbuffer) || !dev->mpeg_user_cnt[vbuffer->tuner_index]) {
            goto out;
        }

        if(vbuffer->remaining[vbuffer->read_idx] >= count) {

            memcpy( &data[saved_cnt - count], &vbuffer->buffers[vbuffer->read_idx][offset], count );

            vbuffer->remaining[vbuffer->read_idx] -= count;
            retval += count;
            done = 1;

        } else if(vbuffer->remaining[vbuffer->read_idx] > 0) { //remaining < count

            memcpy( &data[saved_cnt - count], &vbuffer->buffers[vbuffer->read_idx][offset], vbuffer->remaining[vbuffer->read_idx] );

            retval += vbuffer->remaining[vbuffer->read_idx];
            count -= vbuffer->remaining[vbuffer->read_idx];
            vbuffer->remaining[vbuffer->read_idx] = 0;
        } else {
            if( printk_ratelimit() ) {
                ERROR("mpeg read, bad state");
            }
            retval = 0;
            goto out;
        }

        if(!vbuffer->remaining[vbuffer->read_idx]) {
            vbuffer->read_idx = WRAPPED_PAGE_INDEX( vbuffer, vbuffer->read_idx + 1 );
        }
    }

out:
    return retval;
}



static long ctn91xx_mpeg_ioctl(struct file *filp, uint cmd, ulong arg)
{
    int ret = 0;
    ctn91xx_dev_t* dev = NULL;


    dev = ctn91xx_lookup_dev_from_file( NULL, filp );

    switch(cmd)
    {
        case CTN91XX_IOCTL_MPEG_READ_START:
        {
            int minor = ctn91xx_lookup_minor_from_file(NULL, filp);
            ret = ctn91xx_ioctl_mpeg_read_start(dev, minor, arg);
            break;
        }
        case CTN91XX_IOCTL_MPEG_READ_COMPLETE:
        {
            int minor = ctn91xx_lookup_minor_from_file(NULL, filp);
            ret = ctn91xx_ioctl_mpeg_read_complete(dev, minor, arg);
            break;
        }
        case CTN91XX_IOCTL_MPEG_READ_SETUP:
        {
            int minor = ctn91xx_lookup_minor_from_file(NULL, filp);
            ret = ctn91xx_ioctl_mpeg_read_setup(dev, minor, arg);
            break;
        }
        case CTN91XX_IOCTL_MPEG_SEND:
        {
            int minor = ctn91xx_lookup_minor_from_file(NULL, filp);
            ret = ctn91xx_mpeg_send(dev, minor, arg);
            break;
        }
        case TCGETS:
            //fopen calls this on us...it's the "is_a_tty" call
            //just tell it to go away
            ret = -EINVAL;
            break;
        default:
            ret = -EINVAL;
            WARNING("Unknown ioctl on ctn91xx mpeg device (%d).", cmd);
            break;
    }

    return ret;
}

static long ctn91xx_mpeg_compat_ioctl(struct file *filp, uint cmd, ulong arg)
{
    return ctn91xx_mpeg_ioctl( filp, cmd, arg );
}

static struct file_operations ctn91xx_mpeg_fops = {
    .owner          = THIS_MODULE,
    .open           = ctn91xx_mpeg_open,
    .release        = ctn91xx_mpeg_release,
    .poll           = ctn91xx_mpeg_poll,
    .mmap           = ctn91xx_mpeg_mmap,
    .llseek         = no_llseek,
    .read           = ctn91xx_mpeg_read,
    .unlocked_ioctl = ctn91xx_mpeg_ioctl,
    .compat_ioctl   = ctn91xx_mpeg_compat_ioctl,
};

int ctn91xx_init_mpeg( ctn91xx_dev_t* dev )
{
    int i;
    cdev_init( &dev->mpeg_cdev, &ctn91xx_mpeg_fops );
    dev->mpeg_cdev.owner = THIS_MODULE;
    cdev_add( &dev->mpeg_cdev, MKDEV( CETON_MAJOR, CETON_MINOR( dev->board_number, MPEG_DEVICE_NUMBER ) ), NUM_MPEG_DEVICES );

    for( i=0; i<NUM_MPEG_DEVICES/2; i++ ) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
        device_create( dev->class, NULL, MKDEV( CETON_MAJOR, CETON_MINOR( dev->board_number, 1 + i ) ), "ctn91xx_mpeg%d_%d", dev->board_number, i );
        device_create( dev->class, NULL, MKDEV( CETON_MAJOR, CETON_MINOR( dev->board_number, 7 + i ) ), "ctn91xx_filter%d_%d", dev->board_number, i );
#else
        device_create( dev->class, NULL, MKDEV( CETON_MAJOR, CETON_MINOR( dev->board_number, 1 + i ) ), dev, "ctn91xx_mpeg%d_%d", dev->board_number, i );
        device_create( dev->class, NULL, MKDEV( CETON_MAJOR, CETON_MINOR( dev->board_number, 7 + i ) ), dev, "ctn91xx_filter%d_%d", dev->board_number, i );
#endif
    }
    return 0;
}

void ctn91xx_deinit_mpeg( ctn91xx_dev_t* dev )
{
    int i;
    for( i=0; i<(NUM_MPEG_DEVICES); i++ ) {
        device_destroy( dev->class, MKDEV( CETON_MAJOR, CETON_MINOR( dev->board_number, 1 + i ) ) );
    }
    cdev_del( &dev->mpeg_cdev );
}


int check_drop(vbuffer_t* vbuffer, ctn91xx_dev_t* dev, int new_map_page_index, int num_pages)
{
    int end_of_new_map_index = (new_map_page_index + num_pages) % vbuffer->npages;

    if(!dev->mpeg_user_cnt[vbuffer->tuner_index]) {
        return 0;
    }

    // Checks to see if there is a drop
    // -If new_map_page_index (which is the new write page) is in the range
    //  that is currently being read from, then we will need to drop data.
    // -The following code checks this and also takes into account the
    //  wrapping in the vbuffer
    //
    if(end_of_new_map_index < new_map_page_index) {
        //it wrapped
        return vbuffer->read_idx >= new_map_page_index || vbuffer->read_idx < end_of_new_map_index;
    } else {
        return vbuffer->read_idx >= new_map_page_index && vbuffer->read_idx < end_of_new_map_index;
    }
}

void ctn91xx_vbuffer_print_internal(ctn91xx_dev_t* dev, vbuffer_t* vbuffer)
{
    int last_was_skipped = 0;
    int j = 0;
    for (j = 0; j < vbuffer->npages; j++) {

        if ((j != 0)
                && (j != vbuffer->npages - 1)
                && vbuffer->lock_cnt[j - 1] == vbuffer->lock_cnt[j]
                && vbuffer->lock_cnt[j] == vbuffer->lock_cnt[j + 1]
                && vbuffer->remaining[j - 1] == vbuffer->remaining[j]
                && vbuffer->remaining[j] == vbuffer->remaining[j + 1]
                && j != vbuffer->read_idx
                && j != vbuffer->write_idx
                && j != vbuffer->notify_idx)
        {
            //Skip this entry
            if (!last_was_skipped) {
                INFO("...");
            }
            last_was_skipped = 1;
        } else {
            last_was_skipped = 0;

            INFO( "vbuffer: %03d\t%d\t%d\t%c%c%c", j, vbuffer->lock_cnt[j], vbuffer->remaining[j],
                  j == vbuffer->read_idx ? 'r' : ' ',
                  j == vbuffer->write_idx ? 'w' : ' ',
                  j == vbuffer->notify_idx ? 'n' : ' ');
        }


    }
}

void ctn91xx_mpeg_unmap_page(ctn91xx_dev_t* dev, vbuffer_t* vbuffer,
                             int page_idx, uint16_t bytes_per_page, uint8_t tuner_index)
{

    if(vbuffer->lock_cnt[page_idx] == 0 && vbuffer->dropped[page_idx]) {

        vbuffer->dropped[page_idx] = 0;
        return;

    } else if(vbuffer->lock_cnt[page_idx] == 0) {
        static int first_time = 1;
        if(first_time) {
            ERROR("lock_cnt == 0 before unlock for tuner_index = %d", tuner_index);
            ctn91xx_vbuffer_print_internal(dev, vbuffer);
            first_time = 0;
        }
        vbuffer->read_idx = vbuffer->notify_idx;
        return;
    }

    vbuffer->lock_cnt[page_idx]--;
    if(vbuffer->lock_cnt[page_idx] == 0) {
        vbuffer->remaining[page_idx] = bytes_per_page;
        vbuffer->sizes[page_idx] = bytes_per_page;
    }
}


void ctn91xx_print_vbuffer(ctn91xx_dev_t* dev, vbuffer_t* vbuffer, int tuner_index)
{
    spin_lock_w_flags(&vbuffer->lock);
    ctn91xx_vbuffer_print_internal(dev, vbuffer);
    spin_unlock_w_flags(&vbuffer->lock);
}

int ctn91xx_print_vbuffer_state(ctn91xx_dev_t* dev, unsigned long arg)
{
    vbuffer_t* vbuffer = NULL;
    uint8_t origin = (uint8_t)arg;
    int tuner_index = arg - CTN91XX_ORIGIN_TUNER0;

    mpeg_vbuffer_from_tuner(origin, dev, &vbuffer);

    if (!vbuffer) {
        ERROR("Bad origin");
        return 0;
    }

    INFO("Stream");

    ctn91xx_print_vbuffer(dev, vbuffer, tuner_index);

    if (tuner_index > CTN91XX_ORIGIN_TUNER5) {
        return 0;
    } else {
        int filter_stream_offset = (CTN91XX_ORIGIN_FILTER0 - CTN91XX_ORIGIN_TUNER0);
        tuner_index = arg - CTN91XX_ORIGIN_TUNER0 + filter_stream_offset;

        origin += filter_stream_offset;
        vbuffer = NULL;

        mpeg_vbuffer_from_tuner(origin, dev, &vbuffer);
    }

    INFO("Accompanying Filter");

    if (!vbuffer) {
        ERROR("Bad origin");
        return 0;
    }

    ctn91xx_print_vbuffer(dev, vbuffer, tuner_index);

    return 0;
}

uint8_t ctn91xx_interrupt_mpeg_notify(ctn91xx_dev_t* dev, uint8_t origin,
                                      uint8_t notify_cnt, uint16_t bytes_per_page)
{
    int i;
    vbuffer_t* vbuffer = NULL;
    int stream_index = origin - CTN91XX_ORIGIN_TUNER0;

    mpeg_vbuffer_from_tuner(origin, dev, &vbuffer);

    if(!vbuffer) {
        ERROR("invalid mpeg origin %02x", origin);
        return notify_cnt;
    }

    vbuffer->packet_count += ( notify_cnt * bytes_per_page ) / 188;

    {
        /**
         * This is what happens here:
         * -This function is called when the hardware has finished with
         *  "notify_cnt" worth of pages.
         * -So we start at the index of the notify_idx and unlock/unmap
         *  "notify_cnt" worth of pages
         * -This works in both the case when USE_MPEG_NOTIFY is on, and also
         *  when it is off (in which case this function will be unmapping a
         *  whole bank at a time)
         *
         * -The wake_up_interruptible lets any readers know we have data ready to read
         */
        int cur_notify_idx = vbuffer->notify_idx;
        spin_lock_w_flags(&vbuffer->lock);

        verify_buffer_state(dev, vbuffer, 1);
        for(i=0; i < notify_cnt; i++) {
            int was_a_drop = vbuffer->dropped[cur_notify_idx];

            ctn91xx_mpeg_unmap_page(dev, vbuffer, cur_notify_idx, bytes_per_page, stream_index);
            if( !was_a_drop ) {
                dev->stats.packet_count[vbuffer->tuner_index] += (bytes_per_page / 188);
#if CHECK_NOTIFY_CC
                notify_cc_check( dev, vbuffer->tuner_index, vbuffer->buffers[cur_notify_idx], bytes_per_page);
#endif
            }

            verify_buffer_state(dev, vbuffer, 2);

            if(vbuffer->lock_cnt[cur_notify_idx] == 0) {
                vbuffer->notify_idx = (cur_notify_idx + 1) % vbuffer->npages;
            }

            verify_buffer_state(dev, vbuffer, 3);
            cur_notify_idx = (cur_notify_idx + 1) % vbuffer->npages;
        }
        verify_buffer_state(dev, vbuffer, 4);

#if USE_PCI
        if( vbuffer->rtp_state.running ) {
            schedule_work( &vbuffer->rtp_state.reader );
        }
#endif

        spin_unlock_w_flags(&vbuffer->lock);
    }

    wake_up_interruptible(&dev->mpeg_wait_queues[stream_index]);

    return notify_cnt;
}

void ctn91xx_interrupt_mpeg_dma(ctn91xx_dev_t* dev, uint8_t origin, int bank)
{
    vbuffer_t* vbuffer = NULL;
    int next_write_page_index;
    int new_map_page_index;
    int num_pages;
    int tuner_index;
    int drop_detected = 0;

    mpeg_vbuffer_from_tuner(origin, dev, &vbuffer);

    if(!vbuffer) {
        ERROR("invalid mpeg origin %02x", origin);
        return;
    }

    tuner_index = origin - CTN91XX_ORIGIN_TUNER0;
    num_pages = num_pages_per_bank(dev, tuner_index);

    /**
     * So this is what happens here:
     * -On entering this function the write_idx is at the bank
     *  that we should unmap.
     * -The bank after write_idx (next_write_page_index)
     *  is locked, and the hardware is currently dma'ing into it.
     * -The bank after that one (new_map_page_index) is the one
     *  we need to map and lock now.
     *
     * After you are done locking the new_map_page_index bank,
     * you need to move the write_idx back to point at next_write_page_index
     *
     * To detect when we are dropping data, we need to check whether the
     * read_idx is contained within the range that "new_map_page_index"
     * will cover. If it is, we want to make the hardware just reuse
     * the current sg list pages, until the reader has moved on.
     *
     */

    {
#if !USE_LEON && PRINT_DROP_IN_ISR
        static int drop_ctr = 0;
#endif
        spin_lock_w_flags(&vbuffer->lock);

        verify_buffer_state(dev, vbuffer, 5);
        next_write_page_index = WRAPPED_PAGE_INDEX(vbuffer, vbuffer->write_idx + num_pages);
        new_map_page_index = (next_write_page_index + num_pages) % vbuffer->npages;

        if(check_drop(vbuffer, dev, new_map_page_index, num_pages)) {
            dev->stats.drop_count[tuner_index]++;

            drop_detected = 1;

            if( dev->stats.drop_count[tuner_index] == 100 ) {
                if( printk_ratelimit() ) {
                    printk("(in isr): drop_count %d tuner_index %d\n",
                           dev->stats.drop_count[tuner_index], tuner_index);
                }
            }
        }

        if(!drop_detected) {
            verify_buffer_state(dev, vbuffer, 6);
            //temporarily change write_idx to new_map_page_index
            vbuffer->write_idx = new_map_page_index;

            vbuffer_map_bank(dev, vbuffer, num_pages, bank);

            vbuffer->write_idx = next_write_page_index;
            verify_buffer_state(dev, vbuffer, 7);
        } else {
            //in the drop case:
            //
            //      o leave write_idx where it is
            //      o set notify_idx = write_idx
            //      o copy the pages currently in the OTHER bank into the bank we
            //          are updating
            //      o flag the pages at write_idx "dropped" so unmap doesn't
            //          freak out when notify is called later with new data
            //

            int j;
            int addr_list_offset = get_addr_list_offset(dev, vbuffer->tuner_index, bank);
            verify_buffer_state(dev, vbuffer, 8);

            vbuffer->notify_idx = vbuffer->write_idx;

            for(j=0; j<num_pages; j++) {

                int cur_page_index = WRAPPED_PAGE_INDEX(vbuffer, vbuffer->write_idx + j);
                int map_page_index = (cur_page_index + num_pages) % vbuffer->npages;

                if(is_filter_stream(vbuffer->tuner_index)) {
                    ctn91xx_write32(vbuffer->dma_addrs[map_page_index],
                                    dev->mpeg_filter_dma_base, addr_list_offset + (j * 4));
                } else {
                    ctn91xx_write32(vbuffer->dma_addrs[map_page_index],
                                    dev->mpeg_dma_base, addr_list_offset + (j * 4));
                }
                vbuffer->dropped[cur_page_index] = 1;
            }

            verify_buffer_state(dev, vbuffer, 9);
#if !USE_LEON && PRINT_DROP_IN_ISR
            WARNING("DROP (%d,%d)", vbuffer->tuner_index, drop_ctr++ );
#endif
        }

        spin_unlock_w_flags(&vbuffer->lock);
    }
}

void vbuffer_map_bank(ctn91xx_dev_t* dev, vbuffer_t* vbuffer, int num_pages, int bank)
{
    int j;
    int addr_list_offset = get_addr_list_offset(dev, vbuffer->tuner_index, bank);

    for(j=0; j<num_pages; j++) {

        int write_page_index = WRAPPED_PAGE_INDEX(vbuffer, vbuffer->write_idx + j);

        if(is_filter_stream(vbuffer->tuner_index)) {
            ctn91xx_write32(vbuffer->dma_addrs[write_page_index],
                            dev->mpeg_filter_dma_base, addr_list_offset + (j * 4));
        } else {
            ctn91xx_write32(vbuffer->dma_addrs[write_page_index],
                            dev->mpeg_dma_base, addr_list_offset + (j * 4));
        }
        vbuffer->lock_cnt[write_page_index]++;
    }
}

void ctn91xx_reset_dma_adjustments(ctn91xx_dev_t* dev, uint32_t stream_index)
{
    if(is_filter_stream(stream_index)) {
        ctn91xx_write16(default_mpeg_bytes_per_page_from_stream(stream_index), dev->mpeg_filter_dma_base,
                        MPEG_FILTER_DMA_BYTES_PER_PAGE_BASE + ((stream_index+CTN91XX_ORIGIN_TUNER0-CTN91XX_ORIGIN_FILTER0)*2));
        ctn91xx_write8(default_num_pages_per_notify(stream_index), dev->mpeg_filter_dma_base,
                       MPEG_FILTER_DMA_PAGES_PER_NOTIFY_BASE + ((stream_index+CTN91XX_ORIGIN_TUNER0-CTN91XX_ORIGIN_FILTER0)*8));
    } else {
        ctn91xx_write16(default_mpeg_bytes_per_page_from_stream(stream_index), dev->mpeg_dma_base, MPEG_DMA_BYTES_PER_PAGE_BASE + (stream_index*2));
        ctn91xx_write8(default_num_pages_per_notify(stream_index), dev->mpeg_dma_base, MPEG_DMA_PAGES_PER_NOTIFY_BASE + (stream_index*8));
    }
}


#endif
