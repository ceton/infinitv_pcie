#ifndef CTN91XX_H
#define CTN91XX_H

#include <linux/types.h>

#define PRINT_RW 0

#define USE_MPEG_NOTIFY 1
#define USE_SPIT_I2C 1
#define USE_I2C_BURST 1
#define SPIT_WRITE_BUFFER_SIZE 4096
#define SPIT_READ_BUFFER_SIZE 4096
#define SPIT_HW_BUFFER_SIZE (512)
#define SPIT_BULK_BUFFER_SIZE (2*1024*1024)
#define RPC_BUFFER_SIZE 516
#define READ_FOR_EVERY_WRITE USE_PCI
#define PRINT_DROP_IN_ISR 0

#define CHECK_NOTIFY_CC 0
#if USE_LEON
    #undef CHECK_NOTIFY_CC
    #define CHECK_NOTIFY_CC 0
#endif

#define NUM_RX_BUFFERS 100
#define RX_BUFFER_LENGTH 2048
#define MAX_DESC_PER_INTERFACE 200

//values for which_proc
#define CC_PROC  (1<<0)
#define DRM_PROC (1<<1)
#define DMA_PROC (1<<2)
#define EXT_PROC (1<<3)

#define MPEG_BUFFER_NPAGES 256
#define FILTER_BUFFER_NPAGES 16
#if USE_LEON
    #undef MPEG_BUFFER_NPAGES
    #undef FILTER_BUFFER_NPAGES
    #define MPEG_BUFFER_NPAGES 8
    #define FILTER_BUFFER_NPAGES 16
#endif

#define MAX_NUMBER_DEVICES 16
#if USE_LEON
    #undef MAX_NUMBER_DEVICES
    #define MAX_NUMBER_DEVICES 1
#endif

#define NUM_MPEG_DEVICES 12

#define REQUEST_LONG_TIMEOUT 5100
#define I2C_MSEC_DELAY 2000
#define SPIT_MSEC_DELAY 1000
#define SPI_MSEC_DELAY 1000
#define SCARD_MSEC_DELAY 5100
#define MCARD_MSEC_DELAY 5100
#define RPC_SEND_TIMEOUT 200
#define POWER_SETTING_DELAY 200

#define DEVICE_NAME "ctn91xx"
#define CTRL_DEVICE_NUMBER 0
#define MPEG_DEVICE_NUMBER 1
#define CETON_MAJOR 231
#define CETON_MINOR( top, bot ) ( (((top) << 4)&0xf0) | ((bot)&0x0f) )
#define CETON_MINOR_BOARD_NUMBER( minor ) ( ( (minor) >> 4) & 0x0f )
#define CETON_MINOR_DEVICE_NUMBER( minor ) ( (minor)&0x0f )
#define MKDEV_CTRL( dev ) MKDEV( CETON_MAJOR, CETON_MINOR( dev->board_number, CTRL_DEVICE_NUMBER ) )

#define CTN91XX_SPI_BUFFER_MAX 8

#define CTN91XX_MMAP_IO_OFFSET                   0
#define CTN91XX_MMAP_TRANSLATION_IO_OFFSET       CTN91XX_MMAP_IO_OFFSET+CTN91XX_REG_REGION_SIZE
#define CTN91XX_MMAP_BULK_BUFFER_OFFSET          CTN91XX_MMAP_TRANSLATION_IO_OFFSET+CTN91XX_TRANSLATION_REG_REGION_SIZE
#define CTN91XX_MMAP_SPIT_READ_BUFFER_OFFSET     (CTN91XX_MMAP_BULK_BUFFER_OFFSET + SPIT_BULK_BUFFER_SIZE)
#define CTN91XX_MMAP_SPIT_WRITE_BUFFER_OFFSET    (CTN91XX_MMAP_SPIT_READ_BUFFER_OFFSET + SPIT_READ_BUFFER_SIZE)
#define CTN91XX_MMAP_SPIT_OFFSET_SKIP            (SPIT_READ_BUFFER_SIZE+SPIT_WRITE_BUFFER_SIZE)

/* card inserted types */
#define CARD_TYPE_PCMCIA    0
#define CARD_TYPE_MCARD     1
#define CARD_TYPE_UNKNOWN   2

#define I2C_STATE_WRITE_START       0x00
#define I2C_STATE_WRITE             0x01
#define I2C_STATE_STOPPING_WRITE    0x02
#define I2C_STATE_READ_START        0x03
#define I2C_STATE_READ              0x04
#define I2C_STATE_STOPPING          0x05
#define I2C_STATE_DONE              0x06

#define I2C_ERROR_NO_ERROR          0x00
#define I2C_ERROR_ARBITRATION_LOST  0x01
#define I2C_ERROR_SLAVE_BUSY        0x02
#define I2C_ERROR_TIMED_OUT         0x03
#define I2C_ERROR_INVALID_BUS       0x04
#define I2C_ERROR_NOT_CONNECTED     0x05

#define SPIT_ERROR_NO_ERROR         0x00
#define SPIT_ERROR_TIMED_OUT        0x01
#define SPIT_ERROR_INVALID_BUS      0x02
#define SPIT_ERROR_NOT_CONNECTED    0x03

#include "ctn91xx_ioctl.h"
#include "ctn91xx_registers.h"

#if defined(__KERNEL__) || defined(NT)

#include "ctn91xx_kal.h"
#include "ctn91xx_structs.h"

#define DEBUG 1

#if DEBUG
#define CETON_PRINTF(args...) printk( args )
#else
#define CETON_PRINTF(args...)
#endif

#define ERROR(s, args...)       CETON_PRINTF( KERN_ERR "%s:%i ERROR: (%d) " s "\n", __FUNCTION__, __LINE__, (dev ? dev->board_number : -1), ## args)
#define WARNING(s, args...)     CETON_PRINTF( KERN_WARNING "%s:%i WARNING: (%d) " s "\n", __FUNCTION__, __LINE__, (dev ? dev->board_number : -1), ## args)
#define INFO(s, args...)        CETON_PRINTF( KERN_INFO "%s:%i : (%d) " s "\n", __FUNCTION__, __LINE__, (dev ? dev->board_number : -1), ## args)
#define SUCCESS(s, args...)     CETON_PRINTF( KERN_DEBUG "%s:%i : (%d) " s "\n", __FUNCTION__, __LINE__, (dev ? dev->board_number : -1), ## args)

#define ERROR_INLINE(s, args...)    CETON_PRINTF( KERN_ERR s "", ## args)
#define WARNING_INLINE(s, args...)  CETON_PRINTF( KERN_WARNING s "", ## args)
#define INFO_INLINE(s, args...)     CETON_PRINTF( KERN_DEBUG s "", ## args)
#define SUCCESS_INLINE(s, args...)  CETON_PRINTF( KERN_DEBUG s "", ## args)

#define NOT_IMPLEMENTED( s, args...) CETON_PRINTF( KERN_DEBUG "%s:%i : " "Not Implemented: " s "\n", __FUNCTION__, __LINE__, ## args)
#define QUIETLY_NOT_IMPLEMENTED( s, args...)

#if defined __cplusplus
extern "C" {
#endif

    int ctn91xx_ioctl_handle(uint32_t cmd, unsigned long arg, ctn91xx_dev_t* dev, int compat);

#if defined __cplusplus
}
#endif

#endif //__KERNEL__ || WIN32

#endif //CTN91XX_H
