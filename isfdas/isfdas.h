#ifndef __ISFDAS_H__
#define __ISFDAS_H__

#include <linux/types.h>

struct sensor_record {
	u64 seq;
	u64 timestamp_ns;

	s16 accel_x;
	s16 accel_y;
	s16 accel_z;

	s16 gyro_x;
	s16 gyro_y;
	s16 gyro_z;

	s16 temp_raw;
};



/* Exported by isfdas.c */
int isfdas_submit_record(struct sensor_record *rec);

#endif
