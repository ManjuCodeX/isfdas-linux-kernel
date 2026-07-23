# ISFDAS - Integrated Sensor Flash Data Acquisition System

ISFDAS reads accelerometer/gyroscope data from an MPU6050 sensor over I2C,
stores it on a W25Q64 SPI NOR flash chip, and exposes it to userspace
through a simple character device (`/dev/isfdas0`). It's made up of three
kernel modules and one small userspace tool.

## How it fits together

```
 MPU6050 (I2C)                          W25Q64 (SPI)
 mpu-6050/mpu_6050.c                    w25q64/w25q64.c
       |                                       ^
       | isfdas_submit_record()                | mtd_read/mtd_write
       v                                       |
              isfdas/isfdas.c
      (fifos, /dev/isfdas0, flash bookkeeping)
                     |
                     v
         application/isfdas_app.c
        (latest / stream / stats / flush)
```

- **mpu-6050** — talks to the MPU6050 over I2C, reads a sample on every
  DATA_RDY interrupt, and hands it off to ISFDAS. Fixed at 100 Hz, ±250 dps,
  ±2g.
- **w25q64** — SPI NOR flash driver, registered as an MTD device. ISFDAS
  only ever talks to it through `mtd_read()` / `mtd_write()` / `mtd_erase()`,
  never raw SPI.
- **isfdas** — the middle layer. Keeps the latest sample in RAM, queues
  records into a userspace fifo and a flash fifo, and creates
  `/dev/isfdas0`. Also keeps a small bit of metadata on flash (last
  sequence number, write offset, record count) so it can pick up where it
  left off after a reboot.
- **application** — a command line tool that talks to `/dev/isfdas0`.

## Directory layout

```
.
├── application/     # userspace CLI tool
│   ├── isfdas_app.c
│   └── isfdas_app.h
├── isfdas/          # core driver (fifos, char device, flash bookkeeping)
│   ├── isfdas.c
│   ├── isfdas.h
│   └── Makefile
├── mpu-6050/        # MPU6050 I2C driver
│   ├── isfdas.h
│   ├── Makefile
│   └── mpu_6050.c
└── w25q64/          # W25Q64 SPI/MTD flash driver
    ├── Makefile
    ├── w25q64.c
    └── w25q64.h
```

## Building

Each kernel module has its own `Makefile`. Build them one at a time:

```bash
cd w25q64 && make
cd ../mpu-6050 && make
cd ../isfdas && make
```

Load them in this order, since isfdas depends on the other two being
present:

```bash
insmod w25q64/w25q64.ko
insmod isfdas/isfdas.ko
insmod mpu-6050/mpu_6050.ko
```

Build the userspace tool with plain gcc:

```bash
cd application
gcc isfdas_app.c -o isfdas_app
```

## Using the CLI tool

```bash
./isfdas_app latest   # print the current live snapshot
./isfdas_app stream    # continuously print incoming records
./isfdas_app stats     # print flash write offset / record count / next seq
./isfdas_app flush     # force a flash write right now
```

## Notes

- Sensor rate and range are fixed at compile time (100 Hz, DLPF 42 Hz,
  ±250 dps, ±2g) to keep the driver small and the data format predictable.
- The "latest" reading always comes from RAM. Sensor history is stored in flash..
- Flash metadata is kept in two slots so a power loss mid-write doesn't
  wipe out the last known good state.
