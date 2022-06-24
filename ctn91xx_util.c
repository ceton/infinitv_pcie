#include "ctn91xx_util.h"
#include "ctn91xx_interrupt.h"
#include "ctn91xx_mpeg.h"
#include "ctn91xx_event.h"
#include "ctn91xx_rtp.h"


#if !USE_LEON
#include <asm/div64.h>
#endif

int ctn91xx_board_init(ctn91xx_dev_t* dev)
{
    static int board_ctr = 0;
#if HAS_MPEG_DMA
    uint32_t i;
#endif

    dev->magic = CTN91XX_MAGIC;

    dev->int_enable = 0;
    dev->board_number = board_ctr++;

    INIT_LIST_HEAD(&dev->event_queue);
    INIT_LIST_HEAD(&dev->delayed_event_queue);

    mutex_init(&dev->fd_mutex);
    spin_lock_init(&dev->event_queue_lock);
    spin_lock_init(&dev->delayed_event_queue_lock);



#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
    INIT_WORK( &dev->delayed_event_worker, ctn91xx_delayed_event_worker, &dev->delayed_event_worker );
    INIT_WORK( &dev->delayed_slot_number_worker, ctn91xx_delayed_slot_number_worker, &dev->delayed_slot_number_worker );
#else
    INIT_WORK( &dev->delayed_event_worker, ctn91xx_delayed_event_worker );
    INIT_WORK( &dev->delayed_slot_number_worker, ctn91xx_delayed_slot_number_worker );
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
#else
#endif

    spin_lock_init(&dev->isr_lock);
    spin_lock_init(&dev->device_lock);

    init_waitqueue_head( &dev->msg_buffer_wait_queue );

    dev->rpc_buffer = kmalloc( RPC_BUFFER_SIZE, GFP_KERNEL );

    if( !dev->rpc_buffer ) {
        goto out_of_memory;
    }

#if USE_LEON
#endif

#if HAS_MPEG_DMA

    dev->dma_timeout = 540000; //20ms in 27mhz cycles

    for(i=0; i<NUM_MPEG_DEVICES; i++) {
        int j;
        vbuffer_t* vbuffer = &dev->mpeg_buffer[i];
        vbuffer->dev = dev;
        spin_lock_init(&vbuffer->lock);
        vbuffer->npages = pages_per_mpeg_device(i);
        vbuffer->tuner_index = i;

        vbuffer->read_idx = 0;
        vbuffer->write_idx = 0;
        vbuffer->notify_idx = 0;
 
        vbuffer->buffers = kzalloc(sizeof(void*) * vbuffer->npages, GFP_KERNEL);
        vbuffer->dma_addrs = kmalloc(sizeof(dma_addr_t) * vbuffer->npages, GFP_KERNEL);
        vbuffer->lock_cnt = kmalloc( vbuffer->npages, GFP_KERNEL );
        vbuffer->dropped = kmalloc( vbuffer->npages, GFP_KERNEL );
        vbuffer->remaining = kmalloc( vbuffer->npages * sizeof(uint16_t), GFP_KERNEL );
        vbuffer->sizes = kmalloc(vbuffer->npages * sizeof(uint16_t), GFP_KERNEL);
        vbuffer->pages = kmalloc(vbuffer->npages * sizeof(struct page*), GFP_KERNEL );

        if(!vbuffer->lock_cnt
                || !vbuffer->dropped
                || !vbuffer->remaining
                || !vbuffer->sizes
                || !vbuffer->buffers
                || !vbuffer->dma_addrs
                || !vbuffer->pages) {
            goto out_of_memory;
        }

        for( j=0; j<vbuffer->npages; j++ ) {
#if USE_PCI
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,18,0)
            vbuffer->buffers[j] = pci_alloc_consistent( dev->pdev, PAGE_SIZE, &vbuffer->dma_addrs[j] );
#else
            vbuffer->buffers[j] = dma_alloc_coherent(& dev->pdev->dev, PAGE_SIZE, &vbuffer->dma_addrs[j],GFP_ATOMIC);
#endif
#else
            vbuffer->buffers[j] = kmalloc(PAGE_SIZE, GFP_KERNEL);
            if( vbuffer->buffers[j] ) {
                vbuffer->dma_addrs[j] = virt_to_phys(vbuffer->buffers[j]);
            }
#endif
            if( !vbuffer->buffers[j] ) {
                ERROR("could not alloc mpeg stream %d (%d)", i, j);
                goto out_of_memory;
            }

            vbuffer->pages[j] = virt_to_page(vbuffer->buffers[j]);
            memset(vbuffer->buffers[j], 0xaa, PAGE_SIZE);

            vbuffer->lock_cnt[j] = 0;
            vbuffer->dropped[j] = 0;
            vbuffer->sizes[j] = 0;
            vbuffer->remaining[j] = 0;
        }

        vbuffer->header_buffer = vmalloc( PAGE_SIZE );

        if( !vbuffer->header_buffer ) {
            goto out_of_virtual_memory;
        }

        vbuffer->header_buffer_page = vmalloc_to_page( vbuffer->header_buffer );

#if USE_PCI
        vbuffer->rtp_state.buffer = vmalloc( RTP_SINK_BUFFER_SIZE );

        if( !vbuffer->rtp_state.buffer ) {
            vfree( vbuffer->header_buffer );
            goto out_of_virtual_memory;
        }


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
        INIT_WORK( &vbuffer->rtp_state.reader, ctn91xx_rtp_reader, &vbuffer->rtp_state.reader );
#else
        INIT_WORK( &vbuffer->rtp_state.reader, ctn91xx_rtp_reader );
#endif
#endif
        init_waitqueue_head(&dev->mpeg_wait_queues[i]);
    }
#endif

    init_waitqueue_head(&dev->event_waitqueue);
    return 1;

out_of_memory:
    ERROR("out of memory on board init\n");
    return 0;

#if HAS_MPEG_DMA
out_of_virtual_memory:
    ERROR("out of virtual memory on board init\n");
    return 0;
#endif
}

void ctn91xx_board_uninit_dma_pool(ctn91xx_dev_t* dev)
{
    int i;

    dev->event_user_cnt = 0;
    for(i=0; i<NUM_MPEG_DEVICES; i++) {
        dev->mpeg_user_cnt[i] = 0;
    }

    //wait for the dma's to be done
    //before trying to unmap the pages
    msleep(100);

#if HAS_MPEG_DMA
    for(i=0; i<NUM_MPEG_DEVICES; i++) {
        int num_pages = num_pages_per_bank(dev, i);
        vbuffer_t* vbuffer = &dev->mpeg_buffer[i];
        int last_index_to_unmap = WRAPPED_PAGE_INDEX(vbuffer, vbuffer->write_idx + (2*num_pages));
        int j;

        spin_lock_w_flags(&vbuffer->lock);

        for(j = vbuffer->notify_idx; j != last_index_to_unmap; j = (j+1) % vbuffer->npages) {

            if(vbuffer->lock_cnt[j] > 0) {
                ctn91xx_mpeg_unmap_page(dev, vbuffer, j, 0, i);
            }
        }

        spin_unlock_w_flags(&vbuffer->lock);

        for( j=0; j<vbuffer->npages; j++ ) {
#if USE_PCI
#if LINUX_VERSION_CODE < KERNEL_VERSION(5,18,0)
            pci_free_consistent( dev->pdev, PAGE_SIZE, vbuffer->buffers[j], vbuffer->dma_addrs[j] );
#else
            dma_free_coherent(& dev->pdev->dev, PAGE_SIZE, vbuffer->buffers[j], vbuffer->dma_addrs[j] );
#endif
#else
            kfree( vbuffer->buffers[j] );
#endif
        }

        vfree( vbuffer->header_buffer );

        kfree( vbuffer->buffers );
        kfree( vbuffer->lock_cnt );
        kfree( vbuffer->dropped );
        kfree( vbuffer->remaining );
        kfree( vbuffer->sizes );
        kfree( vbuffer->pages );
        kfree( vbuffer->dma_addrs );
    }
#endif
}

void ctn91xx_board_uninit(ctn91xx_dev_t* dev)
{
    ctn91xx_event_cleanup_waiting(dev);

#if USE_LEON
#endif

    if( dev->rpc_buffer ) {
        kfree( dev->rpc_buffer );
        dev->rpc_buffer = NULL;
    }

    ctn91xx_board_uninit_dma_pool(dev);
}

#if PRINT_RW
ctn91xx_dev_t* gdev = NULL;
ctn91xx_set_dev(ctn91xx_dev_t* dev) {
    gdev = dev;
}
#endif

void ctn91xx_orin8(uint8_t data, void* __iomem base, unsigned long offset)
{
    BUG_ON(!base);
    ctn91xx_write8( (ctn91xx_read8(base, offset) | data), base, offset );
}

void ctn91xx_andin8(uint8_t data, void* __iomem base, unsigned long offset)
{
    BUG_ON(!base);
    ctn91xx_write8( (ctn91xx_read8(base, offset) & data), base, offset );
}

void ctn91xx_orin32(uint32_t data, void* __iomem base, unsigned long offset)
{
    BUG_ON(!base);
    ctn91xx_write32( (ctn91xx_read32(base, offset) | data), base, offset );
}

void ctn91xx_andin32(uint32_t data, void* __iomem base, unsigned long offset)
{
    BUG_ON(!base);
    ctn91xx_write32( (ctn91xx_read32(base, offset) & data), base, offset );
}


void ctn91xx_write8(uint8_t byte, void __iomem* base, unsigned long offset)
{
    BUG_ON(!base);
    if (base != 0) {
#if PRINT_RW
        INFO("w8 0x%08x 0x%02x", base+offset, byte);
        ctn91xx_busywait(gdev, 1000);
#endif
#if USE_PCI
        writeb(byte, ((uint8_t*)base)+offset);
#if READ_FOR_EVERY_WRITE
        readb(((uint8_t*)base)+offset);
#endif
#elif USE_LEON
        __raw_writeb( byte, ((uint8_t*)base)+offset );
#endif
    }
}

void ctn91xx_write16(uint16_t data, void __iomem* base, unsigned long offset)
{
    BUG_ON(!base);
    if (base != 0) {
#if PRINT_RW
        INFO("w16 0x%08x 0x%02x", base+offset, data);
        ctn91xx_busywait(gdev, 1000);
#endif
#if USE_PCI
        writew(data, ((uint8_t*)base)+offset);
#if READ_FOR_EVERY_WRITE
        readw(((uint8_t*)base)+offset);
#endif
#elif USE_LEON
        __raw_writew( data, ((uint8_t*)base)+offset );
#endif
    }
}

void ctn91xx_write32(uint32_t data, void __iomem* base, unsigned long offset)
{
    BUG_ON(!base);
    if (base != 0) {
#if PRINT_RW
        INFO("w32 0x%08x 0x%02x", base+offset, data);
        ctn91xx_busywait(gdev, 1000);
#endif
#if USE_PCI
        writel(data, ((uint8_t*)base)+offset);
#if READ_FOR_EVERY_WRITE
        readl(((uint8_t*)base)+offset);
#endif
#elif USE_LEON
        __raw_writel( data, ((uint8_t*)base)+offset );
#endif
    }
}

#if USE_LEON
void ctn91xx_write64(uint64_t data, void __iomem* base, unsigned long offset)
{
    BUG_ON(!base);
    if (base != 0) {
#if PRINT_RW
        INFO("w64 0x%08x 0x%02x", base+offset, data);
        ctn91xx_busywait(gdev, 1000);
#endif
        writell(data, ((uint8_t*)base)+offset);
#if READ_FOR_EVERY_WRITE
        data = readll(((uint8_t*)base)+offset);
#endif
    }
}
#endif

uint8_t ctn91xx_read8(void __iomem* base, unsigned long offset)
{
    BUG_ON(!base);
    if (base != 0) {
#if PRINT_RW
        INFO("r8 0x%08x", base+offset);
        ctn91xx_busywait(gdev, 1000);
#endif
#if USE_PCI
        return readb(((uint8_t*)base)+offset);
#elif USE_LEON
        return __raw_readb(((uint8_t*)base)+offset);
#endif
    } else {
        ctn91xx_dev_t* dev = NULL;
        ERROR("read from unmapped memory");
        return 0xff;
    }
}

uint16_t ctn91xx_read16(void __iomem* base, unsigned long offset)
{
    BUG_ON(!base);
    if (base != 0) {
#if PRINT_RW
        INFO("r16 0x%08x", base+offset);
        ctn91xx_busywait(gdev, 1000);
#endif
#if USE_PCI
        return readw(((uint8_t*)base)+offset);
#elif USE_LEON
        return __raw_readw(((uint8_t*)base)+offset);
#endif
    } else {
        ctn91xx_dev_t* dev = NULL;
        ERROR("read from unmapped memory");
        return 0xffff;
    }
}

uint32_t ctn91xx_read32(void __iomem* base, unsigned long offset)
{
    BUG_ON(!base);
    if (base != 0) {
#if PRINT_RW
        INFO("r32 0x%08x", base+offset);
        ctn91xx_busywait(gdev, 1000);
#endif
#if USE_PCI
        return readl(((uint8_t*)base)+offset);
#elif USE_LEON
        return __raw_readl(((uint8_t*)base)+offset);
#endif
    } else {
        ctn91xx_dev_t* dev = NULL;
        ERROR("read from unmapped memory");
        return 0xffffffff;
    }
}

uint64_t ctn91xx_read64(void __iomem* base, unsigned long offset)
{
    BUG_ON(!base);
    if (base != 0) {
        return readll(((uint8_t*)base)+offset);
    } else {
        ctn91xx_dev_t* dev = NULL;
        ERROR("read from unmapped memory");
        return 0xffffffffffffffffULL;
    }
}

uint16_t ctn91xx_release_version(ctn91xx_dev_t* dev) {

    return dev->release_version;
}

uint16_t ctn91xx_chip_version(ctn91xx_dev_t* dev) {

    return dev->chip_version;
}

//deprecated
uint32_t ctn91xx_board_version(ctn91xx_dev_t* dev) {

    return dev->board_version;
}

uint32_t ctn91xx_which_proc(ctn91xx_dev_t* dev)
{
    return DMA_PROC | DRM_PROC | CC_PROC | EXT_PROC;
    return dev->which_proc;
}

uint64_t ctn91xx_gettick(ctn91xx_dev_t* dev)
{
    return ctn91xx_read64(dev->system_control_base, SYSTEM_CONTROL_UP_TIME_OFFSET);
}

uint64_t ctn91xx_up_time(ctn91xx_dev_t* dev)
{
    uint64_t cur_tick = ctn91xx_gettick(dev);
    uint64_t ns;
#if USE_LEON
    ns = (cur_tick / (uint64_t)dev->ticks_per_us)*1000ULL;
#else
    do_div(cur_tick, (uint64_t)dev->ticks_per_us);
    ns = cur_tick*1000ULL;
#endif
    return ns;
}

void ctn91xx_busywait(ctn91xx_dev_t* dev, uint32_t us) {
    uint64_t ns = us*1000ULL;
    uint64_t s = ctn91xx_up_time(dev);
    uint64_t e;
    do
    {
        e = ctn91xx_up_time(dev);
    } while(e - s < ns);
}

void ctn91xx_mod_sdump_buffer( const char * module_name, const char * function, int line, const uint8_t* buf, uint32_t length, const char* name)
{
    int i, j, i_valid;
    u32 remainder = (length % 16);
    u32 top_len = length + (remainder ? 16 - remainder : 0);
    u32 char_index;
    u8 c;

    if (name) {
        printk("%s:%i DUMP: \"%s\" (%p) length %d\n", function, line, name, buf, length);
    } else {
        printk("%s:%i DUMP: (%p) length %d\n", function, line, buf, length);
    }

    printk("%08d  ", 0);

    for(i=0; i<top_len; i++) {

        i_valid = (i < length);

        if (i_valid) {
            printk("%02x ", buf[i]);
        } else {
            printk("   ");
        }

        if(i % 16 == 15 && i != 0) {
            printk("  |");
            j = 0;
            for(j=0; j<16; j++) {
                char_index = i - 15 + j;
                if (char_index < length) {
                    c = buf[char_index];
                    printk("%c", (c >= '0' && c <= 'z') ? c : '.');
                } else {
                    printk("%c", ' ');
                }
            }
            if (i == top_len - 1) {
                printk("|");
            } else {
                printk("|\n%08d  ", i+1);
            }

        } else if (i % 8 == 7) {
            printk(" ");
        }


    }
    printk( "\n");
}

void ctn91xx_mod_pci_cfg_sdump_buffer( const char * module_name, const char * function, int line, struct pci_dev* pdev, int offset, uint32_t length, const char* name)
{
    int i, j, i_valid;
    u32 remainder = (length % 16);
    u32 top_len = length + (remainder ? 16 - remainder : 0);
    u32 char_index;
    u8 c;
    u8 b;

    if (name) {
        printk("%s:%i DUMP: \"%s\" (%d) length %d\n", function, line, name, offset, length);
    } else {
        printk("%s:%i DUMP: (%d) length %d\n", function, line, offset, length);
    }

    printk("%08d  ", 0);

    for(i=0; i<top_len; i++) {

        i_valid = (i < length);

        if (i_valid) {
            pci_read_config_byte(pdev, i, &b);
            printk("%02x ", b);
        } else {
            printk("   ");
        }

        if(i % 16 == 15 && i != 0) {
            printk("  |");
            j = 0;
            for(j=0; j<16; j++) {
                char_index = i - 15 + j;
                if (char_index < length) {
                    pci_read_config_byte(pdev, char_index, &c);
                    printk("%c", (c >= '0' && c <= 'z') ? c : '.');
                } else {
                    printk("%c", ' ');
                }
            }
            if (i == top_len - 1) {
                printk("|");
            } else {
                printk("|\n%08d  ", i+1);
            }

        } else if (i % 8 == 7) {
            printk(" ");
        }


    }
    printk( "\n");
}

uint8_t solstice_bus_id(uint8_t i2c_addr) {

    switch(i2c_addr >> 1) {
        case 0x61:
            return 0; //First Xceive
        case 0x64:
            return 1; //Second Xceive
        case 0x40:
        case 0x47:
        case 0x26:
            return 2; //Auvitek's and CPLD
        case 0x29:
            return 3; //DRXj
    }

    return 0; //doesn't really matter what we pick cause nothing will respond anyway
}

#if USE_LEON
#endif
