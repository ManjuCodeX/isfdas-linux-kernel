#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/mtd/mtd.h>

#include "w25q64.h"

struct wq_prv
{
    struct spi_device *spi;
    struct mtd_info mtd;
};

// functions used below, defined later in the file
static int read_data(struct spi_device *spi, u32 addr, u8 *buf, size_t len);
static int write_data(struct spi_device *spi, u32 addr, u8 *buf, size_t len);
static int erase_sector(struct spi_device *spi, u32 addr);

// reads some bytes from the flash chip starting at addr
static int read_data(struct spi_device *spi, u32 addr, u8 *buf, size_t len)
{
    u8 tx[4];
    int ret;

    if (addr > END_ADD)
        return -EINVAL;

    if (len == 0 || len > W25Q64_PAGE_SIZE)
        return -EINVAL;

    tx[0] = W25Q64_CMD_READ_DATA;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;

    ret = spi_write_then_read(spi, tx, sizeof(tx), buf, len);
    if (ret)
        return ret;

    return 0;
}

// writes some bytes to one page of the flash chip
static int write_data(struct spi_device *spi, u32 addr, u8 *buf, size_t len)
{
    struct spi_transfer xfer[2] = {};
    struct spi_message msg;
    u8 header[4];
    u8 en_cmd = W25Q64_CMD_WRITE_ENABLE;
    u8 status_cmd = W25Q64_CMD_READ_STATUS1;
    u8 status;
    int ret;

    if ((addr + len) > W25Q64_FLASH_SIZE)
        return -EINVAL;

    // a page write cannot go past the end of its own page, so we check
    // that here just in case the caller forgot to split it up
    if ((addr % W25Q64_PAGE_SIZE) + len > W25Q64_PAGE_SIZE)
        return -EINVAL;

    // tell the chip we want to write
    ret = spi_write(spi, &en_cmd, 1);
    if (ret)
        return ret;

    // send the write command + address, then the actual data
    header[0] = W25Q64_CMD_PAGE_PROGRAM;
    header[1] = (addr >> 16) & 0xFF;
    header[2] = (addr >> 8) & 0xFF;
    header[3] = addr & 0xFF;

    spi_message_init(&msg);

    xfer[0].tx_buf = header;
    xfer[0].len = sizeof(header);

    xfer[1].tx_buf = buf;
    xfer[1].len = len;

    spi_message_add_tail(&xfer[0], &msg);
    spi_message_add_tail(&xfer[1], &msg);

    ret = spi_sync(spi, &msg);
    if (ret)
        return ret;

    // keep checking the status register until the chip is done writing
    do
    {
        ret = spi_write_then_read(spi, &status_cmd, 1, &status, 1);
        if (ret)
            return ret;
    } while (status & W25Q64_SR_BUSY);

    return 0;
}

// erases one sector (block) of the flash chip
static int erase_sector(struct spi_device *spi, u32 addr)
{
    u8 en_cmd = W25Q64_CMD_WRITE_ENABLE;
    u8 tx[4];
    u8 status;
    u8 status_cmd = W25Q64_CMD_READ_STATUS1;
    int ret;

    if (addr >= W25Q64_FLASH_SIZE)
        return -EINVAL;

    if (addr % W25Q64_SECTOR_SIZE)
        return -EINVAL;

    ret = spi_write(spi, &en_cmd, 1);
    if (ret)
        return ret;

    tx[0] = W25Q64_CMD_SECTOR_ERASE;
    tx[1] = (addr >> 16) & 0xFF;
    tx[2] = (addr >> 8) & 0xFF;
    tx[3] = addr & 0xFF;

    ret = spi_write(spi, tx, sizeof(tx));
    if (ret)
        return ret;

    // wait for the erase to finish before returning
    do
    {
        ret = spi_write_then_read(spi, &status_cmd, 1, &status, 1);
        if (ret)
            return ret;
    } while (status & W25Q64_SR_BUSY);

    return 0;
}

// reads the chip ID and prints it, just to check we are talking to
// the right chip
static int w25q64_read_id(struct spi_device *spi)
{
    u8 cmd = W25Q64_CMD_READ_JEDEC_ID;
    u8 id[3] = {0};
    int ret;

    ret = spi_write_then_read(spi, &cmd, 1, id, 3);
    if (ret)
        return ret;

    dev_info(&spi->dev, "JEDEC ID: %02x %02x %02x\n", id[0], id[1], id[2]);

    if (id[0] != W25Q64_MANUFACTURER_ID)
        dev_warn(&spi->dev, "unexpected manufacturer ID — check wiring/CS/mode\n");

    return 0;
}

// mtd read callback, just calls read_data() in a loop since read_data()
// can only handle one page at a time
static int w25q64_mtd_read(struct mtd_info *mtd, loff_t from, size_t len,
                           size_t *retlen, u_char *buf)
{
    struct wq_prv *prv = mtd->priv;
    int ret;

    if (from + len > mtd->size)
        return -EINVAL;

    *retlen = 0;
    while (*retlen < len)
    {
        size_t chunk = min_t(size_t, len - *retlen, W25Q64_PAGE_SIZE);

        ret = read_data(prv->spi, from + *retlen, buf + *retlen, chunk);
        if (ret)
            return ret;

        *retlen += chunk;
    }

    return 0;
}

// mtd write callback, splits the buffer into page-sized pieces since
// write_data() will not write across a page boundary
static int w25q64_mtd_write(struct mtd_info *mtd, loff_t to, size_t len,
                            size_t *retlen, const u_char *buf)
{
    struct wq_prv *prv = mtd->priv;
    int ret;

    if (to + len > mtd->size)
        return -EINVAL;

    *retlen = 0;
    while (*retlen < len)
    {
        loff_t addr = to + *retlen;
        size_t space_in_page = W25Q64_PAGE_SIZE - (addr % W25Q64_PAGE_SIZE);
        size_t chunk = min_t(size_t, len - *retlen, space_in_page);

        ret = write_data(prv->spi, addr, (u8 *)buf + *retlen, chunk);
        if (ret)
            return ret;

        *retlen += chunk;
    }

    return 0;
}

// mtd erase callback, erases sector by sector until the whole range
// is done
static int w25q64_mtd_erase(struct mtd_info *mtd, struct erase_info *instr)
{
    struct wq_prv *prv = mtd->priv;
    u32 addr = instr->addr;
    u32 len = instr->len;
    int ret;

    if (addr % mtd->erasesize || len % mtd->erasesize)
        return -EINVAL;

    if (addr + len > mtd->size)
        return -EINVAL;

    while (len)
    {
        ret = erase_sector(prv->spi, addr);
        if (ret)
        {
            instr->fail_addr = addr;
            return ret;
        }

        addr += mtd->erasesize;
        len -= mtd->erasesize;
    }

    return 0;
}

// called when the spi device is found, sets everything up and
// registers the mtd device
static int spi_probe(struct spi_device *spi)
{
    struct wq_prv *my_data;
    int ret;

    dev_info(&spi->dev,
             "mode=%u bits=%u speed=%uHz cs=%u\n",
             spi->mode,
             spi->bits_per_word,
             spi->max_speed_hz,
             spi->chip_select);

    my_data = devm_kzalloc(&spi->dev, sizeof(*my_data), GFP_KERNEL);
    if (!my_data)
        return -ENOMEM;

    my_data->spi = spi;
    spi_set_drvdata(spi, my_data);

    spi->mode = SPI_MODE_0;
    spi->bits_per_word = 8;

    ret = spi_setup(spi);
    if (ret)
        return ret;

    ret = w25q64_read_id(spi);
    if (ret)
        return ret;

    my_data->mtd.name = "w25q64";
    my_data->mtd.owner = THIS_MODULE;
    my_data->mtd.dev.parent = &spi->dev;
    my_data->mtd.type = MTD_NORFLASH;
    my_data->mtd.flags = MTD_CAP_NORFLASH;
    my_data->mtd.size = W25Q64_FLASH_SIZE;
    my_data->mtd.erasesize = W25Q64_SECTOR_SIZE;
    my_data->mtd.writesize = 1;
    my_data->mtd.writebufsize = W25Q64_PAGE_SIZE;
    my_data->mtd._read = w25q64_mtd_read;
    my_data->mtd._write = w25q64_mtd_write;
    my_data->mtd._erase = w25q64_mtd_erase;
    my_data->mtd.priv = my_data;

    ret = mtd_device_register(&my_data->mtd, NULL, 0);
    if (ret)
    {
        dev_err(&spi->dev, "mtd_device_register failed: %d\n", ret);
        return ret;
    }

    dev_info(&spi->dev, "W25Q64 MTD driver probed\n");
    return 0;
}

// called when the device is removed, just unregisters the mtd device
static void spi_remove(struct spi_device *spi)
{
    struct wq_prv *my_data = spi_get_drvdata(spi);

    mtd_device_unregister(&my_data->mtd);
}

static const struct of_device_id storage[] = {
    {.compatible = "winbond,w25q64"},
    {}};
MODULE_DEVICE_TABLE(of, storage);

static struct spi_driver w25q64_driver = {
    .driver = {
        .name = "w25q64",
        .of_match_table = storage,
    },
    .probe = spi_probe,
    .remove = spi_remove,
};

module_spi_driver(w25q64_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Manjunatha V");
MODULE_DESCRIPTION("W25Q64 SPI NOR flash MTD driver");
