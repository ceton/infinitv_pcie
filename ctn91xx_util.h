#ifndef CTN91XX_UTIL_H
#define CTN91XX_UTIL_H

#include "ctn91xx.h"


int ctn91xx_board_init(ctn91xx_dev_t* dev);
void ctn91xx_board_uninit(ctn91xx_dev_t* dev);

uint16_t ctn91xx_release_version(ctn91xx_dev_t* dev);
uint16_t ctn91xx_chip_version(ctn91xx_dev_t* dev);
uint32_t ctn91xx_board_version(ctn91xx_dev_t* dev); //deprecated

void ctn91xx_write8(uint8_t byte, void __iomem* base, unsigned long offset);
void ctn91xx_write16(uint16_t data, void __iomem* base, unsigned long offset);
void ctn91xx_write32(uint32_t data, void __iomem* base, unsigned long offset);
void ctn91xx_write64(uint64_t data, void __iomem* base, unsigned long offset);

uint8_t ctn91xx_read8(void __iomem* base, unsigned long offset);
uint16_t ctn91xx_read16(void __iomem* base, unsigned long offset);
uint32_t ctn91xx_read32(void __iomem* base, unsigned long offset);
uint64_t ctn91xx_read64(void __iomem* base, unsigned long offset);

void ctn91xx_orin8(uint8_t data, void* __iomem base, unsigned long offset);
void ctn91xx_andin8(uint8_t data, void* __iomem base, unsigned long offset);
void ctn91xx_orin32(uint32_t data, void* __iomem base, unsigned long offset);
void ctn91xx_andin32(uint32_t data, void* __iomem base, unsigned long offset);

void ctn91xx_face_gpio_read_write_cycle( ctn91xx_dev_t* dev );
void ctn91xx_face_gpio_read_write_cycle_ex( ctn91xx_dev_t* dev, uint8_t print );

uint32_t ctn91xx_origin_is_tuner(uint8_t o);
uint32_t ctn91xx_interrupt_is_error(uint8_t type);

uint32_t ctn91xx_which_proc(ctn91xx_dev_t* dev);

uint8_t solstice_bus_id(uint8_t i2c_addr);

uint64_t ctn91xx_gettick(ctn91xx_dev_t* dev);
uint64_t ctn91xx_up_time(ctn91xx_dev_t* dev);
void ctn91xx_busywait(ctn91xx_dev_t* dev, uint32_t us);

void ctn91xx_bandwidth_test(ctn91xx_dev_t* dev);
void ctn91xx_network_bandwidth_test(ctn91xx_dev_t* dev, struct file* out_file);


#define sdump_buffer(buffer, length, name) ctn91xx_mod_sdump_buffer( "ctn91xx", __FUNCTION__, __LINE__, (buffer), (length), (name))
void ctn91xx_mod_sdump_buffer( const char * module_name, const char * function, int line, const uint8_t* buf, uint32_t length, const char* name);

#define pci_cfg_sdump_buffer(pdev, offset, length, name) ctn91xx_mod_pci_cfg_sdump_buffer( "ctn91xx", __FUNCTION__, __LINE__, (pdev), (offset), (length), (name))
void ctn91xx_mod_pci_cfg_sdump_buffer( const char * module_name, const char * function, int line, struct pci_dev* pdev, int offset, uint32_t length, const char* name);

int descriptor_pool_create(ctn91xx_eth_t* eth);
void descriptor_pool_destroy(ctn91xx_eth_t* eth);
dma_desc_t* descriptor_pool_get(ctn91xx_eth_t* eth, int for_tx);
void descriptor_pool_put(ctn91xx_eth_t* eth, dma_desc_t* desc, int for_tx);

void print_desc_list(ctn91xx_dev_t* dev, struct list_head* list, const char* name);

#endif
