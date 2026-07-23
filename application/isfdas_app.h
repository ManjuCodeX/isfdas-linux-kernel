#ifndef ISFDAS_APP_H
#define ISFDAS_APP_H

#include <stdint.h>
#include <sys/ioctl.h>

/* Device node */
#define ISFDAS_DEVICE_PATH "/dev/isfdas0"

/*
 * Sensor record
 * Must match the kernel driver's structure exactly.
 */
struct sensor_record {
    uint64_t seq;
    uint64_t timestamp_ns;

    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t temp_raw;
};

/*
 * Flash statistics
 * Must match the kernel driver's structure exactly.
 */
struct isfdas_flash_stats {
    uint64_t next_seq;
    uint32_t write_offset;
    uint32_t record_count;
};

/* IOCTL definitions */
#define ISFDAS_IOC_MAGIC              'I'

#define ISFDAS_IOC_GET_LATEST \
    _IOR(ISFDAS_IOC_MAGIC, 1, struct sensor_record)

#define ISFDAS_IOC_FLUSH \
    _IO(ISFDAS_IOC_MAGIC, 2)

#define ISFDAS_IOC_GET_FLASH_STATS \
    _IOR(ISFDAS_IOC_MAGIC, 3, struct isfdas_flash_stats)

#endif /* ISFDAS_APP_H */