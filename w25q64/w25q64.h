

#ifndef _W25Q64_H_
#define _W25Q64_H_

#include <linux/types.h>
#include <linux/ioctl.h>
#include <linux/bitops.h>

/* 
 * Address range (for sanity checks)
*/
#define START_ADD 0x000000
#define END_ADD   0x7FFFFF

/* 
 * Flash Size
*/
#define W25Q64_FLASH_SIZE          (8 * 1024 * 1024)    /* 8MB */
#define W25Q64_PAGE_SIZE           256                  /* 256 Bytes */
#define W25Q64_SECTOR_SIZE         (4 * 1024)           /* 4KB */
#define W25Q64_BLOCK_SIZE          (64 * 1024)          /* 64KB */

/* Geometry */
#define W25Q64_TOTAL_PAGES         (W25Q64_FLASH_SIZE / W25Q64_PAGE_SIZE)
#define W25Q64_TOTAL_SECTORS       (W25Q64_FLASH_SIZE / W25Q64_SECTOR_SIZE)
#define W25Q64_TOTAL_BLOCKS        (W25Q64_FLASH_SIZE / W25Q64_BLOCK_SIZE)

/* Address */
#define W25Q64_ADDRESS_BYTES       3                   /* 24-bit address */

/* 
 * Standard SPI Commands
*/
#define W25Q64_CMD_WRITE_ENABLE    0x06
#define W25Q64_CMD_WRITE_DISABLE   0x04

#define W25Q64_CMD_READ_STATUS1    0x05
#define W25Q64_CMD_READ_STATUS2    0x35

#define W25Q64_CMD_READ_DATA       0x03
#define W25Q64_CMD_FAST_READ       0x0B

#define W25Q64_CMD_PAGE_PROGRAM    0x02

#define W25Q64_CMD_SECTOR_ERASE    0x20
#define W25Q64_CMD_BLOCK_ERASE32K  0x52
#define W25Q64_CMD_BLOCK_ERASE64K  0xD8
#define W25Q64_CMD_CHIP_ERASE      0xC7

#define W25Q64_CMD_READ_JEDEC_ID   0x9F

#define W25Q64_CMD_RESET_ENABLE    0x66
#define W25Q64_CMD_RESET_DEVICE    0x99

/* 
  Status Register Bits
*/
#define W25Q64_SR_BUSY             BIT(0)
#define W25Q64_SR_WEL              BIT(1)


#define W25Q64_MANUFACTURER_ID     0xEF
#define W25Q64_MEMORY_TYPE         0x40
#define W25Q64_CAPACITY_ID         0x17


struct flash_data {
	__u32 addr;
	__u32 len;
	__u8  buf[256];
};


#define W25Q64_IOC_MAGIC   'w'

#define READ_DATA   _IOWR(W25Q64_IOC_MAGIC, 1, struct flash_data)
#define WRITE_DATA  _IOW(W25Q64_IOC_MAGIC,  2, struct flash_data)
#define ERASE_DATA  _IOW(W25Q64_IOC_MAGIC,  3, struct flash_data)

#endif 

