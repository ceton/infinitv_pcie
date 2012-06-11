#include "ctn91xx_pci.h"
#include "ctn91xx_util.h"
#include "ctn91xx_interrupt.h"
#include "ctn91xx_mpeg.h"
#include "ctn91xx_net.h"
#include "ctn91xx_reset.h"
#include "ctn91xx_driver.h"

#if USE_PCI

#define MAX_BOARDS 20
static int boards[MAX_BOARDS] = {};

static int face_present = 0;

static int ctn91xx_register(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    uint32_t freq;
    ctn91xx_dev_t* dev = kzalloc( sizeof( ctn91xx_dev_t ), GFP_KERNEL );

    if(!dev) {
        return -ENOMEM;
    }
    
    dev->pdev = pdev;

    if(!ctn91xx_board_init(dev)) {
        return -ENOMEM;
    }

    if(pci_enable_device(pdev) != 0) {
        ERROR("cannot enable pci device");
        return -ENODEV;
    }

    pci_set_drvdata(pdev, (void*)dev);

    dev->irq = pdev->irq;

    if(pci_request_regions(pdev, "ctn91xx") != 0) {
        pci_disable_device(pdev);
        ERROR("pci device busy");
        return -EBUSY;
    }

    
    /* Try to init DMA */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,1,0)
    if(pci_set_dma_mask(pdev, DMA_32BIT_MASK)) {
#else
    if(pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
#endif
        ERROR("No suitable DMA mask available.");
    }

    pci_set_master(pdev);

    dev->hw_reg_base = pci_resource_start(pdev, 0);
    dev->base = ioremap_nocache(dev->hw_reg_base, CTN91XX_REG_REGION_SIZE);

    if(!dev->base) {
        ERROR("reg ioremap failed");
        return -EIO;
    }

    INFO("IO hw_reg_base address: %lx", dev->hw_reg_base);
    INFO("reg_base: %p", dev->base);

    dev->translation_hw_reg_base = pci_resource_start(pdev, 1);
    dev->translation_base = ioremap_nocache(dev->translation_hw_reg_base, CTN91XX_TRANSLATION_REG_REGION_SIZE);

    if(!dev->translation_base) {
        WARNING("translation reg ioremap failed");
    } else {
        INFO("IO translation_hw_reg_base address: %lx", dev->translation_hw_reg_base);
        INFO("translation_reg_base: %p", dev->translation_base);
    }

    dev->mpeg_filter_dma_base = dev->base + FILTER_DMA_REG_OFFSET;
    dev->mpeg_dma_base =        dev->base + DMA_REG_OFFSET;
    dev->system_control_base =  dev->base + SYSTEM_CONTROL_OFFSET;
    dev->msg_base =             dev->base + MSG_BUFFER_OFFSET;
    dev->timer_reg_base =       dev->base + TIMER_REG_OFFSET;


    dev->release_version = ctn91xx_read16(dev->system_control_base, CTN91XX_RELEASE_VERSION_OFFSET);
    dev->chip_version = ctn91xx_read16(dev->system_control_base, CTN91XX_CHIP_VERSION_OFFSET);

    if(dev->release_version == 0 || dev->chip_version == 0) {

        WARNING("read versions %d and %d, retrying", dev->release_version, dev->chip_version);
        dev->release_version = ctn91xx_read16(dev->system_control_base, CTN91XX_RELEASE_VERSION_OFFSET);
        dev->chip_version = ctn91xx_read16(dev->system_control_base, CTN91XX_CHIP_VERSION_OFFSET);
    }

    if( dev->release_version == 0xffff ) {
        ERROR("chip not configured %04x", dev->release_version);
        goto reg_base_cleanup;
    }

    freq = ctn91xx_read32(dev->system_control_base, SYSTEM_CONTROL_CLOCK_FREQ);
    dev->ticks_per_us = (freq / 1000000);
    INFO("system clock freq %d, ticks per us %d", freq, dev->ticks_per_us);
    if( dev->ticks_per_us == 100 ) {
        //FIXME: acting like noah based on clock frequency
        dev->is_noah = 1;
    }

    if( dev->is_noah ) {
        //overwrite board_number with correct value from hw, if it supports it
        //must be done before device files are registered
        uint32_t tag = ctn91xx_read32( dev->msg_base, MSG_BUFFER_TAG );
        uint8_t retries_left = 2;
        uint8_t valid = ( tag >> 16 ) & 0x1;
        dev->board_number = ( tag >> 8 ) & 0xff;

        //0 is the reset value, and a valid board number (d'oh!)
        //so try to ensure we got the right one
        if( dev->board_number == 0 ) {
            while( !valid && retries_left > 0 ) {
                msleep(1000);
                tag = ctn91xx_read32( dev->msg_base, MSG_BUFFER_TAG );
                valid = ( tag >> 16 ) & 0x1;
                dev->board_number = ( tag >> 8 ) & 0xff;
                retries_left--;
            }
        }

        dev->board_booting = valid;

        if( valid && boards[0] == 0 && dev->board_number == 0 ) {
            INFO("face detected as present");
            face_present = 1;
        }

        dev->face_present = face_present;

        if( !dev->face_present ) {
            //will be set to next available by code below
            dev->board_number = 0;
        }
    }

    if( dev->board_number > MAX_BOARDS ) {
        ERROR("invalid board number %d", dev->board_number);
        dev->board_number = MAX_BOARDS;
    }

    if( !boards[dev->board_number] ) {
        boards[dev->board_number] = 1;
    } else {
        int i;
        for( i=0; i<MAX_BOARDS; i++ ) {
            if( !boards[i] ) {
                dev->board_number = i;
                boards[i] = 1;
                break;
            }
        }
    }

    if(dev->release_version != 0 && dev->chip_version != 0) {

        dev->which_proc = ctn91xx_read32(dev->base, PROC_IDENT_OFFSET);
        ctn91xx_reset_all(dev);

        INFO("Driver initialization successful for CTN-%d v%d", dev->chip_version, dev->release_version);
    } else {
        WARNING("release_version %d chip_version %d", dev->release_version, dev->chip_version);
    }

    if( ctn91xx_register_ctrl_device( dev ) ) {
        ERROR("Unable to register char device");
        goto reg_base_cleanup;
    }

#if HAS_MPEG_DMA
    if( ctn91xx_init_mpeg(dev) < 0 ) {
        goto reg_base_cleanup;
    }
#endif

    if( ctn91xx_net_init( dev ) ) {
        ERROR("failed to init net device");
        goto dev_cleanup;
    }

    /* Now enable our interrupt */
    dev->int_enable = 1;
    INFO("irq: %d", pdev->irq);
    if(request_irq(pdev->irq, ctn91xx_isr, IRQF_SHARED, DEVICE_NAME, dev)) {
        ERROR("Cannot allocate irq %d", pdev->irq);
        pdev->irq = 0;
        goto net_cleanup;
    }

    return 0;

net_cleanup:
    ctn91xx_net_deinit( dev );
dev_cleanup:
    ctn91xx_unregister_ctrl_device( dev );
#if HAS_MPEG_DMA
    ctn91xx_deinit_mpeg(dev);
#endif

reg_base_cleanup:
    if(dev->translation_base) {
        iounmap(dev->translation_base);
    }
    iounmap(dev->base);

    pci_release_regions(pdev);
    pci_disable_device(pdev);
    return -EIO;
}

static void __devexit ctn91xx_unregister(struct pci_dev *pdev)
{
    ctn91xx_dev_t* dev = (ctn91xx_dev_t*)pci_get_drvdata(pdev);

    free_irq(pdev->irq, dev);
    dev->int_enable = 0;


    ctn91xx_write8(0x0, dev->mpeg_dma_base, MPEG_DMA_INTERRUPT_ENABLE);
    ctn91xx_write8(0x1, dev->mpeg_dma_base, MPEG_DMA_RESET);

    msleep(1);

    if(ctn91xx_read8(dev->mpeg_dma_base, MPEG_DMA_BUSY)) {
        ERROR("resetting while dma is busy, this is BAD");
    }

    ctn91xx_net_deinit( dev );

    iounmap(dev->base);

    if(dev->translation_base) {
        iounmap(dev->translation_base);
    }

    ctn91xx_unregister_ctrl_device( dev );

#if HAS_MPEG_DMA
    ctn91xx_deinit_mpeg(dev);
#endif

    pci_release_regions(pdev);

    pci_disable_device(pdev);
    ctn91xx_board_uninit(dev);

    kfree( dev );
}

static struct pci_device_id ctn91xx_table[] __devinitdata = {
    { CETON_VENDOR_ID, CTN91XX_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    { ALTERA_VENDOR_ID, CTN91XX_DEVICE_ID, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0 },
    { 0, }
};

static struct pci_driver ctn91xx_driver = {
    .name = DEVICE_NAME,
    .id_table = ctn91xx_table,
    .probe = ctn91xx_register,
    .remove = __devexit_p(ctn91xx_unregister),
};

int ctn91xx_register_pci_driver()
{
    return pci_register_driver(&ctn91xx_driver);
}

void ctn91xx_unregister_pci_driver()
{
    pci_unregister_driver(&ctn91xx_driver);
}

MODULE_DEVICE_TABLE(pci, ctn91xx_table);

#endif
