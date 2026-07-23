/* 
 * isfdas_app.c - simple command line tool to talk to /dev/isfdas0
 *
 * commands: latest, stream, stats, flush
 *
 * build with: gcc isfdas_app.c -o isfdas_app
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include "isfdas_app.h"



// prints one record, used by both the "latest" and "stream" commands.
// the "\033[K" part just clears any leftover text at the end of the
// line so old numbers don't stick around when we redraw
static void print_record(const struct sensor_record *rec)
{
	printf("SEQ=%-12llu\033[K\n", (unsigned long long)rec->seq);
	printf("TIME=%-15llu\033[K\n", (unsigned long long)rec->timestamp_ns);
	printf("AX=%-8d\033[K\n", rec->accel_x);
	printf("AY=%-8d\033[K\n", rec->accel_y);
	printf("AZ=%-8d\033[K\n", rec->accel_z);
	printf("GX=%-8d\033[K\n", rec->gyro_x);
	printf("GY=%-8d\033[K\n", rec->gyro_y);
	printf("GZ=%-8d\033[K\n", rec->gyro_z);
	printf("TEMP=%-8d\033[K\n", rec->temp_raw);
}

// how many lines print_record() prints, need this to know how far
// to move the cursor back up when redrawing
#define RECORD_PRINT_LINES 9

// opens the device, grabs the latest record with an ioctl, prints
// it, then closes the device
static int cmd_latest(void)
{
	int fd;
	struct sensor_record rec;
	int ret = 0;

	fd = open(ISFDAS_DEVICE_PATH, O_RDONLY);
	if (fd < 0) {
		perror("isfdas_app: latest: open(" ISFDAS_DEVICE_PATH ")");
		return -1;
	}

	memset(&rec, 0, sizeof(rec));

	if (ioctl(fd, ISFDAS_IOC_GET_LATEST, &rec) < 0) {
		if (errno == ENODATA)
			fprintf(stderr,
				"isfdas_app: latest: no record available yet (ENODATA)\n");
		else
			perror("isfdas_app: latest: ioctl(ISFDAS_IOC_GET_LATEST)");
		ret = -1;
		goto out_close;
	}

	printf("---- Latest Live Record ----\n");
	print_record(&rec);

out_close:
	close(fd);
	return ret;
}

// opens the device and keeps reading records in a loop, printing
// each one in place so it looks like the numbers are just updating
// instead of scrolling. needs a real terminal to look right
static int cmd_stream(void)
{
	int fd;
	struct sensor_record rec;
	ssize_t n;
	unsigned long count = 0;

	fd = open(ISFDAS_DEVICE_PATH, O_RDONLY);
	if (fd < 0) {
		perror("isfdas_app: stream: open(" ISFDAS_DEVICE_PATH ")");
		return -1;
	}

	printf("---- Streaming records from %s (Ctrl+C to stop) ----\n",
	       ISFDAS_DEVICE_PATH);

	// print an empty placeholder block first so we have something
	// to move the cursor back up into on the first real record
	memset(&rec, 0, sizeof(rec));
	printf("Record #%-8lu\033[K\n", count);
	print_record(&rec);

	for (;;) {
		n = read(fd, &rec, sizeof(rec));

		if (n < 0) {
			if (errno == EINTR)
				continue; /* interrupted by a signal, retry */
			perror("isfdas_app: stream: read");
			close(fd);
			return -1;
		}

		if (n == 0) {
			fprintf(stderr,
				"isfdas_app: stream: read() returned 0 (unexpected EOF)\n");
			break;
		}

		if (n != (ssize_t)sizeof(rec)) {
			fprintf(stderr,
				"isfdas_app: stream: short read (%zd of %zu bytes), stopping\n",
				n, sizeof(rec));
			close(fd);
			return -1;
		}

		count++;

		// move cursor back up to the top of the block and redraw
		// everything again with the new values
		printf("\033[%dA", RECORD_PRINT_LINES + 1);
		printf("Record #%-8lu\033[K\n", count);
		print_record(&rec);
	}

	close(fd);
	return 0;
}

// opens the device, asks for flash stats with an ioctl, prints them
static int cmd_stats(void)
{
	int fd;
	struct isfdas_flash_stats stats;
	int ret = 0;

	fd = open(ISFDAS_DEVICE_PATH, O_RDONLY);
	if (fd < 0) {
		perror("isfdas_app: stats: open(" ISFDAS_DEVICE_PATH ")");
		return -1;
	}

	memset(&stats, 0, sizeof(stats));

	if (ioctl(fd, ISFDAS_IOC_GET_FLASH_STATS, &stats) < 0) {
		perror("isfdas_app: stats: ioctl(ISFDAS_IOC_GET_FLASH_STATS)");
		ret = -1;
		goto out_close;
	}

	printf("---- Flash Stats ----\n");
	printf("Next Sequence: %llu\n", (unsigned long long)stats.next_seq);
	printf("Write Offset:  %u\n", stats.write_offset);
	printf("Record Count:  %u\n", stats.record_count);

out_close:
	close(fd);
	return ret;
}

// opens the device and tells it to flush the flash fifo right now
static int cmd_flush(void)
{
	int fd;
	int ret = 0;

	fd = open(ISFDAS_DEVICE_PATH, O_RDONLY);
	if (fd < 0) {
		perror("isfdas_app: flush: open(" ISFDAS_DEVICE_PATH ")");
		return -1;
	}

	if (ioctl(fd, ISFDAS_IOC_FLUSH, NULL) < 0) {
		perror("isfdas_app: flush: ioctl(ISFDAS_IOC_FLUSH)");
		ret = -1;
		goto out_close;
	}

	printf("Flush succeeded: flash_fifo drained via ISFDAS_IOC_FLUSH.\n");

out_close:
	close(fd);
	return ret;
}

// just prints how to use this program
static void usage(const char *progname)
{
	fprintf(stderr,
		"Usage: %s <command>\n"
		"\n"
		"Commands:\n"
		"  latest   Print the current live snapshot (ISFDAS_IOC_GET_LATEST)\n"
		"  stream   Continuously read and print records from %s\n"
		"  stats    Print flash bookkeeping stats (ISFDAS_IOC_GET_FLASH_STATS)\n"
		"  flush    Force an immediate flash-fifo drain (ISFDAS_IOC_FLUSH)\n",
		progname, ISFDAS_DEVICE_PATH);
}

// checks the command typed in and runs the matching function
int main(int argc, char *argv[])
{
	if (argc != 2) {
		usage(argv[0]);
		return EXIT_FAILURE;
	}

	if (strcmp(argv[1], "latest") == 0)
		return cmd_latest() == 0 ? EXIT_SUCCESS : EXIT_FAILURE;

	if (strcmp(argv[1], "stream") == 0)
		return cmd_stream() == 0 ? EXIT_SUCCESS : EXIT_FAILURE;

	if (strcmp(argv[1], "stats") == 0)
		return cmd_stats() == 0 ? EXIT_SUCCESS : EXIT_FAILURE;

	if (strcmp(argv[1], "flush") == 0)
		return cmd_flush() == 0 ? EXIT_SUCCESS : EXIT_FAILURE;

	fprintf(stderr, "isfdas_app: unknown command '%s'\n\n", argv[1]);
	usage(argv[0]);
	return EXIT_FAILURE;
}