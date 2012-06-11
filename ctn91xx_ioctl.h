#ifndef CTN91XX_IOCTL_H
#define CTN91XX_IOCTL_H

#define SIOCTL(n) (100+n)

/* user space ioctl's */
#define CTN91XX_IOCTL_RESET_ALL              SIOCTL(6)
#define CTN91XX_IOCTL_SET_ALWAYS_SCARD       SIOCTL(7)
#define CTN91XX_IOCTL_DOWNLOAD_FW_TO_MEMORY  SIOCTL(8)
#define CTN91XX_IOCTL_MPEG_READ_SETUP        SIOCTL(9)
#define CTN91XX_IOCTL_MPEG_READ_START        SIOCTL(10)
#define CTN91XX_IOCTL_MPEG_READ_COMPLETE     SIOCTL(11)
#define CTN91XX_IOCTL_VBUFFER_STATUS         SIOCTL(12)
#define CTN91XX_IOCTL_SET_SLOT_NUMBER        SIOCTL(13)
#define CTN91XX_IOCTL_EVENT_READ             SIOCTL(14)
#define CTN91XX_IOCTL_I2C_WRITE_READ         SIOCTL(15)
#define CTN91XX_IOCTL_DMA_STATS              SIOCTL(16)
#define CTN91XX_IOCTL_MPEG_SEND              SIOCTL(17)
#define CTN91XX_IOCTL_RPC_SEND               SIOCTL(18)
#define CTN91XX_IOCTL_SPIT_WRITE_READ        SIOCTL(19)
// 20 unused
#define CTN91XX_IOCTL_GPIO_READ_WRITE        SIOCTL(21)
#define CTN91XX_IOCTL_ENABLE_POWER_SETTING   SIOCTL(22)
#define CTN91XX_IOCTL_GET_STATS              SIOCTL(23)
#define CTN91XX_IOCTL_GPIO_RAW_SETUP         SIOCTL(24)
#define CTN91XX_IOCTL_IS_BOARD_BOOTED        SIOCTL(25)
#define CTN91XX_IOCTL_SET_FACE_ATTACHED      SIOCTL(26)
#define CTN91XX_IOCTL_NOTIFY_MASK_TABLE      SIOCTL(27)
#define CTN91XX_IOCTL_PRINT_DESC_LIST        SIOCTL(28)
#define CTN91XX_IOCTL_FORCE_TX_IRQ           SIOCTL(29)
#define CTN91XX_IOCTL_FORCE_RX_IRQ           SIOCTL(30)
#define CTN91XX_IOCTL_PRINT_NET_STATS        SIOCTL(31)
#define CTN91XX_IOCTL_GET_SLOT_NUMBER        SIOCTL(32)
#define CTN91XX_IOCTL_GET_FACE_PRESENT        SIOCTL(33)

typedef struct {
    uint32_t count;
    uint32_t amount_last_read;
} mpeg_read_t;

typedef struct {
    uint32_t count;
    int32_t out_fd;
    uint8_t* header;
    uint32_t header_length;
} mpeg_send_t;

typedef struct {
    uint32_t count;
    int32_t out_fd;
    uint32_t header;
    uint32_t header_length;
} mpeg_send_compat_t;

typedef struct {
    uint32_t npages;
    uint32_t read_index;
    uint16_t bytes_per_page;
} mpeg_stream_setup_t;

typedef struct {
    uint8_t* data;
    uint32_t count;
} event_read_t;

typedef struct {
    uint32_t data;
    uint32_t count;
} event_read_compat_t;

typedef struct {
    uint8_t error;//must be first
    uint16_t read_addr;
    uint32_t read_len;
    uint16_t write_addr;
    uint32_t write_len;
    uint8_t bus;
} i2c_write_read_t;

typedef struct {
    uint8_t error;//must be first
    uint8_t bus;
    uint8_t bucket;
    uint32_t read_len;
    uint32_t write_len;
} spit_write_read_t;

typedef struct {
    uint8_t tuner;
    uint32_t packet_count;
} dma_stats_t;

typedef struct {
    uint8_t payload_start;
    uint32_t section_length;
    uint16_t length;
    uint8_t* msg;
} rpc_send_t;

typedef struct {
    uint8_t payload_start;
    uint32_t section_length;
    uint16_t length;
    uint32_t msg;
} rpc_send_compat_t;

typedef struct {
    uint32_t fw_len;
    uint8_t* fw;
} fw_t;

typedef struct {
    uint8_t pin;
    uint8_t value;
    uint8_t is_read;
} gpio_read_write_t;

typedef struct {
    uint32_t is_input;
    uint32_t mask;
} gpio_raw_setup_t;

typedef struct {
    uint8_t enable;
    uint8_t type;
} enable_power_setting_t;

typedef struct {
    uint32_t packet_count[NUM_MPEG_DEVICES];
    uint32_t drop_count[NUM_MPEG_DEVICES];
    uint32_t discont_count[NUM_MPEG_DEVICES];
    uint32_t mismatched_notify_clear[NUM_MPEG_DEVICES];
    uint32_t isr_count;
    uint32_t isr_mpeg_count;
    uint32_t isr_timer_count;
    uint32_t isr_cablecard_count;
    uint32_t isr_spit_count;
    uint32_t isr_drm_count;
    uint32_t isr_gpio_count;
    uint32_t isr_scte21_count;
    uint32_t isr_rpc_count;
    uint32_t isr_net_count;
    uint32_t isr_eth_count;
    uint32_t isr_not_handled;
    uint32_t scte21_checksum_failed_count;
    uint32_t scte21_slow_count;
    uint32_t scte21_overflow_count;
    uint32_t scte21_continue_count;
    uint32_t isr_net_rx_count;
    uint32_t isr_net_tx_count;
    uint32_t isr_net_zero_count;
} ctn91xx_stats_t;

#endif
