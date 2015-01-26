#include "ctn91xx.h"
#include "ctn91xx_interrupt.h"
#include "ctn91xx_ioctl.h"
#include "ctn91xx_util.h"
#include "ctn91xx_driver.h"
#include "ctn91xx_mpeg.h"
#include "ctn91xx_event.h"

#if USE_PCI
#include "ctn91xx_pci.h"
#endif


ctn91xx_dev_t* ctn91xx_lookup_dev_from_file(struct inode* inode, struct file* file)
{
    if(file->private_data) {
        ctn91xx_dev_fd_state_t* fd_state = (ctn91xx_dev_fd_state_t*)file->private_data;
        return fd_state->dev;
    } else if(inode) {
        ctn91xx_dev_fd_state_t* fd_state = kmalloc( sizeof( ctn91xx_dev_fd_state_t ), GFP_KERNEL );

        if( !fd_state ) {
            return NULL;
        }

        if( CETON_MINOR_DEVICE_NUMBER( iminor(inode) ) == 0 ) {
            fd_state->dev = container_of( inode->i_cdev, ctn91xx_dev_t, ctrl_cdev );
        } else {
            fd_state->dev = container_of( inode->i_cdev, ctn91xx_dev_t, mpeg_cdev );
        }
        fd_state->minor = iminor(inode);
        file->private_data = fd_state;
        return fd_state->dev;
    } else {
        BUG_ON(!inode);
        return NULL;
    }
}

int ctn91xx_lookup_minor_from_file(struct inode* inode, struct file* file)
{
    if(inode) {
        return iminor(inode);
    } else if(file && file->private_data) {
        ctn91xx_dev_fd_state_t* fd_state = (ctn91xx_dev_fd_state_t*)file->private_data;
        return fd_state->minor;
    } else {
        BUG();
        return -1;
    }
}

void ctn91xx_cleanup_dev_from_file(struct inode* inode, struct file* file)
{
    if(file->private_data) {
        kfree(file->private_data);
    }
}

static int ctn91xx_open(struct inode * inode, struct file * file)
{
    int ret = 0;
    ctn91xx_dev_t* dev = NULL;

    dev = ctn91xx_lookup_dev_from_file(inode, file);

    mutex_lock(&dev->fd_mutex);
    dev->event_user_cnt++;
    if(dev->event_user_cnt == 1) {
        dev->always_scard = 0;
    }
    mutex_unlock(&dev->fd_mutex);

    return ret;
}

static int ctn91xx_release(struct inode * inode, struct file * file)
{
    ctn91xx_dev_t* dev = NULL;

    dev = ctn91xx_lookup_dev_from_file(inode, file);

    mutex_lock(&dev->fd_mutex);
    dev->event_user_cnt--;
    if(dev->event_user_cnt == 0) {

        ctn91xx_event_cleanup_waiting(dev);
    }
    mutex_unlock(&dev->fd_mutex);

    ctn91xx_cleanup_dev_from_file(inode, file);

    return 0;
}


static long ctn91xx_ioctl(struct file *filp, uint cmd, ulong arg)
{
    ctn91xx_dev_t* dev = ctn91xx_lookup_dev_from_file( NULL, filp );
    return ctn91xx_ioctl_handle( cmd, arg, dev, 0 );
}

static long ctn91xx_compat_ioctl(struct file *filp, uint cmd, ulong arg)
{
    ctn91xx_dev_t* dev = ctn91xx_lookup_dev_from_file( NULL, filp );
    return ctn91xx_ioctl_handle( cmd, arg, dev, 1 );
}



static int ctn91xx_mmap(struct file* filp, struct vm_area_struct* vma)
{
    int ret = 0;
    ctn91xx_dev_t* dev = NULL;
    unsigned long pfn = 0;
    unsigned long size = vma->vm_end - vma->vm_start;
    uint32_t page_offset_addr = vma->vm_pgoff * PAGE_SIZE;

    dev = ctn91xx_lookup_dev_from_file(NULL, filp);

    switch( page_offset_addr ) {
        case CTN91XX_MMAP_IO_OFFSET:
        {
            if( size > CTN91XX_REG_REGION_SIZE ) {
                return -EINVAL;
            }

            pfn = (unsigned long)dev->hw_reg_base >> PAGE_SHIFT;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
            vma->vm_flags |= VM_IO | VM_RESERVED;
#else
            vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
#endif

            if(io_remap_pfn_range(vma, vma->vm_start,
                                  pfn,
                                  vma->vm_end - vma->vm_start,
                                  vma->vm_page_prot)) {
                ERROR("failed to mmap pfn=0x%08lx -> vm_start=0x%08lx, vm_end=0x%08lx", pfn, vma->vm_start, vma->vm_end);
                return -EINVAL;
            }
            break;
        }
        case CTN91XX_MMAP_TRANSLATION_IO_OFFSET:
        {
            if( size > CTN91XX_TRANSLATION_REG_REGION_SIZE ) {
                return -EINVAL;
            }

            pfn = (unsigned long)dev->translation_hw_reg_base >> PAGE_SHIFT;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0)
            vma->vm_flags |= VM_IO | VM_RESERVED;
#else
            vma->vm_flags |= VM_IO | VM_DONTEXPAND | VM_DONTDUMP;
#endif

            if(io_remap_pfn_range(vma, vma->vm_start,
                                  pfn,
                                  vma->vm_end - vma->vm_start,
                                  vma->vm_page_prot)) {
                ERROR("failed to mmap pfn=0x%08lx -> vm_start=0x%08lx, vm_end=0x%08lx", pfn, vma->vm_start, vma->vm_end);
                return -EINVAL;
            }
            break;
            break;
        }
        default:
        {
            ERROR("unknown mmap offset %d", page_offset_addr );
            return -EINVAL;
        }
    }

    return ret;
}

static struct file_operations ctn91xx_ctl_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = ctn91xx_ioctl,
    .compat_ioctl   = ctn91xx_compat_ioctl,
    .open           = ctn91xx_open,
    .release        = ctn91xx_release,
    .mmap           = ctn91xx_mmap,
    .poll           = ctn91xx_event_poll,
    .llseek         = no_llseek,
    .read           = ctn91xx_event_read
};

static struct class* ctn91xx_class;

int ctn91xx_register_ctrl_device( ctn91xx_dev_t* dev )
{
    cdev_init( &dev->ctrl_cdev, &ctn91xx_ctl_fops );
    dev->ctrl_cdev.owner = THIS_MODULE;
    cdev_add( &dev->ctrl_cdev, MKDEV_CTRL( dev ), 1 );

    dev->class = ctn91xx_class;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,27)
    device_create( dev->class, NULL, MKDEV_CTRL( dev ), "ctn91xx_ctl%d", dev->board_number );
#else
    device_create( dev->class, NULL, MKDEV_CTRL( dev ), dev, "ctn91xx_ctl%d", dev->board_number );
#endif
    return 0;
}

void ctn91xx_unregister_ctrl_device( ctn91xx_dev_t* dev )
{
    device_destroy( dev->class, MKDEV_CTRL( dev ) );
    cdev_del( &dev->ctrl_cdev );
}

static int __init ctn91xx_init(void)
{
    int ret;

    ctn91xx_dev_t* dev = NULL;

    ret = register_chrdev_region( MKDEV( CETON_MAJOR, 0 ), 255 , DEVICE_NAME );

    if( ret != 0 ) {
        return ret;
    }

    ctn91xx_class = class_create( THIS_MODULE, DEVICE_NAME );

    if( IS_ERR( ctn91xx_class ) ) {
        ERROR("failed to create class");
        unregister_chrdev_region( MKDEV( CETON_MAJOR, 0 ), 255 );
        return PTR_ERR( ctn91xx_class );
    }

#if USE_PCI
    ret = ctn91xx_register_pci_driver();
#endif


    return ret;
}

static void __exit ctn91xx_exit(void)
{
#if USE_PCI
    ctn91xx_unregister_pci_driver();
#endif


    class_destroy( ctn91xx_class );
    unregister_chrdev_region( MKDEV( CETON_MAJOR, 0 ), 255 );
}

module_init(ctn91xx_init);
module_exit(ctn91xx_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Austin Foxley");
MODULE_DESCRIPTION("Ceton HW Driver");

