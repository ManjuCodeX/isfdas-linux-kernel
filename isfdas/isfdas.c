// SPDX-License-Identifier: GPL-2.0
/*
 * isfdas.c - ISFDAS main driver
 *
 * This driver sits in the middle. It takes data from the MPU6050 sensor
 * driver and saves it, and also passes it to the W25Q64 flash driver to
 * store on flash. It does not talk to I2C or SPI itself, it just manages
 * the data and queues.
 *
 * How it works:
 *   MPU6050 driver calls isfdas_submit_record()
 *      -> saves latest record in RAM (for quick read)
 *      -> puts record into two queues (fifos):
 *           1) user_fifo  -> read by userspace using read()
 *           2) flash_fifo -> read by a workqueue which writes to flash
 *
 * Userspace can use /dev/isfdas0 with:
 *   read()  - get new records (blocks if none available, unless O_NONBLOCK)
 *   poll()  - wait until new data is ready
 *   ioctl() - get latest snapshot, get flash stats, or force a flush
 *
 * Notes:
 *   - This driver only uses the flash driver through MTD functions
 *     (mtd_read/mtd_write), never touches SPI directly.
 *   - Live/latest data always comes from RAM.
 *   - History data is meant to come from flash (W25Q64 MTD device).
 *   - timestamp_ns is set by the MPU6050 driver, not by us.
 *   - seq number is given by us here, not by the producer.
 *   - user_fifo and flash_fifo are separate so one slow reader does not
 *     block the other one.
 *
 * Author: ISFDAS project
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/workqueue.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/export.h>
#include <linux/atomic.h>
#include <linux/ioctl.h>
#include <linux/mtd/mtd.h>
#include <linux/crc32.h>
#include "isfdas.h"

#define ISFDAS_MODULE_NAME "isfdas"
#define ISFDAS_DEVICE_NAME "isfdas0"
#define ISFDAS_CLASS_NAME "isfdas"

/* size of the two fifos, must be power of 2 for kfifo */
#define ISFDAS_USER_FIFO_DEPTH 128
#define ISFDAS_FLASH_FIFO_DEPTH 128

#define ISFDAS_META_BATCH 50 /* only write meta to flash every 50 records */

#define ISFDAS_FLASH_META_SLOT0_OFFSET 0
#define ISFDAS_FLASH_META_SLOT1_OFFSET W25Q64_SECTOR_SIZE
#define ISFDAS_FLASH_RECORD_OFFSET (2U * W25Q64_SECTOR_SIZE)


/*
 * This struct is saved on flash. It tells us where we stopped writing
 * last time, so after reboot we know where to continue instead of
 * starting from zero again.
 */
#define ISFDAS_FLASH_META_MAGIC 0x49534644U /* "ISFD" */
#define ISFDAS_FLASH_META_VERSION 1

struct isfdas_flash_meta
{
	u32 magic;
	u32 version;
	u64 last_seq;	  /* last seq number we saved to flash */
	u32 write_offset; /* next place to write in record area */
	u32 record_count; /* how many valid records we have right now */
	u32 crc;		  /* checksum of the above fields */
};

/*
 * How big our record area on flash is. Later this should be read from
 * the mtd_info of the flash chip instead of hardcoding it here.
 */
// #define ISFDAS_FLASH_RECORD_AREA_SIZE (8U * 1024U * 1024U)
#define ISFDAS_FLASH_RECORD_AREA_SIZE ((8U * 1024U * 1024U) - (2U * W25Q64_SECTOR_SIZE))

/*
 * ioctl commands for this driver. Normally these go in a uapi header
 * so userspace can share them, but keeping them here for now.
 */
struct isfdas_flash_stats
{
	u64 next_seq;
	u32 write_offset;
	u32 record_count;
};

#define ISFDAS_IOC_MAGIC 'I'
#define ISFDAS_IOC_GET_LATEST _IOR(ISFDAS_IOC_MAGIC, 1, struct sensor_record)
#define ISFDAS_IOC_FLUSH _IO(ISFDAS_IOC_MAGIC, 2)
#define ISFDAS_IOC_GET_FLASH_STATS _IOR(ISFDAS_IOC_MAGIC, 3, struct isfdas_flash_stats)

// #define ISFDAS_FLASH_META_OFFSET 0
// #define ISFDAS_FLASH_RECORD_OFFSET W25Q64_SECTOR_SIZE

#define W25Q64_SECTOR_SIZE (4 * 1024) /* 4KB */

/*
 * Main driver struct. Only one of these exists for the whole system,
 * so we just keep a single global pointer to it.
 */
struct isfdas_dev
{
	struct cdev cdev;
	struct class *class;
	dev_t devt;
	int major;

	/* latest record, always kept up to date */
	struct mutex latest_lock;
	struct sensor_record latest_record;
	bool has_record;

	/* seq number counter, goes up by 1 every record */
	atomic64_t next_seq;

	/* fifo for userspace read() calls */
	struct kfifo user_fifo;
	spinlock_t user_fifo_lock;

	/* fifo for the flash-writing workqueue */
	struct kfifo flash_fifo;
	spinlock_t flash_fifo_lock;

	/* keeps track of where we are writing on flash */
	struct mutex flash_meta_lock;
	u32 flash_write_offset;
	u32 flash_record_count;

	/* used to wake up userspace when new data comes in */
	wait_queue_head_t read_wq;

	/* workqueue that actually writes to flash */
	struct workqueue_struct *wq;
	struct work_struct flash_work;

	u32 flash_meta_slot;	/* 0 or 1, which meta sector is active */
	u32 commits_since_meta; /* count records since last meta write */
};

/* the one and only device instance */
static struct isfdas_dev *isfdas_dev_ptr;

/*
 * Below are the functions that talk to the flash chip through MTD.
 * We never do raw SPI calls here, only mtd_read/mtd_write/mtd_erase
 * on the mtd_info that the W25Q64 driver already registered.
 */

/*
 * isfdas_flash_store() - write one record to flash
 *
 * returns int so later we can report an error if write/erase fails.
 */
static int isfdas_flash_store(struct isfdas_dev *idev, struct sensor_record *rec)
{
	struct mtd_info *mtd;
	struct erase_info erase = {};
	size_t retlen;
	loff_t addr;
	int ret;

	mtd = get_mtd_device_nm("w25q64");
	if (IS_ERR(mtd))
		return PTR_ERR(mtd);

	/* figure out where this record should go */
	addr = ISFDAS_FLASH_RECORD_OFFSET + idev->flash_write_offset;

	/* if we are at the start of a sector, erase it first before writing */
	if ((idev->flash_write_offset % mtd->erasesize) == 0)
	{
		erase.addr = addr;
		erase.len = mtd->erasesize;
		ret = mtd_erase(mtd, &erase);
		if (ret)
		{
			put_mtd_device(mtd);
			return ret;
		}
	}

	ret = mtd_write(mtd, addr, sizeof(*rec), &retlen, (u_char *)rec);
	put_mtd_device(mtd);

	if (ret)
		return ret;
	if (retlen != sizeof(*rec))
		return -EIO;

	return 0;
}
/*
 * isfdas_flash_read_meta_slot() - read one meta sector from flash
 */
static int isfdas_flash_read_meta_slot(u32 offset, struct isfdas_flash_meta *meta)
{
	struct mtd_info *mtd;
	size_t retlen;
	int ret;

	mtd = get_mtd_device_nm("w25q64");
	if (IS_ERR(mtd))
		return PTR_ERR(mtd);

	ret = mtd_read(mtd, offset, sizeof(*meta), &retlen, (u_char *)meta);
	put_mtd_device(mtd);

	if (ret)
		return ret;
	if (retlen != sizeof(*meta))
		return -EIO;

	return 0;
}

/*
 * isfdas_flash_read_meta() - read both meta sectors and pick the good one
 *
 * We keep 2 copies (slot 0 and slot 1) so if power goes off while
 * writing one, we still have the other one to fall back on. A slot is
 * only good if magic, version, and checksum all match.
 */
static int isfdas_flash_read_meta(struct isfdas_dev *idev,
								  struct isfdas_flash_meta *meta)
{
	struct isfdas_flash_meta slot0, slot1;
	u32 crc0, crc1;
	bool v0, v1;

	v0 = !isfdas_flash_read_meta_slot(ISFDAS_FLASH_META_SLOT0_OFFSET, &slot0) &&
		 slot0.magic == ISFDAS_FLASH_META_MAGIC &&
		 slot0.version == ISFDAS_FLASH_META_VERSION;
	if (v0) {
		crc0 = crc32(0, (const void *)&slot0,
					offsetof(struct isfdas_flash_meta, crc));
		v0 = (crc0 == slot0.crc);
	}

	v1 = !isfdas_flash_read_meta_slot(ISFDAS_FLASH_META_SLOT1_OFFSET, &slot1) &&
		 slot1.magic == ISFDAS_FLASH_META_MAGIC &&
		 slot1.version == ISFDAS_FLASH_META_VERSION;
	if (v1) {
		crc1 = crc32(0, (const void *)&slot1,
					offsetof(struct isfdas_flash_meta, crc));
		v1 = (crc1 == slot1.crc);
	}

	if (!v0 && !v1)
		return -ENODATA; /* both slots are empty/bad, treat as fresh flash */

	/* pick whichever slot has the higher last_seq (the newer one) */
	if (v0 && (!v1 || slot0.last_seq >= slot1.last_seq))
	{
		*meta = slot0;
		idev->flash_meta_slot = 0; /* next write goes to slot 1 */
	}
	else
	{
		*meta = slot1;
		idev->flash_meta_slot = 1; /* next write goes to slot 0 */
	}

	return 0;
}

/*
 * isfdas_flash_write_meta() - save updated meta info to flash
 *
 * We flip between slot 0 and slot 1 each time so we are not erasing
 * the same sector over and over.
 */

static int isfdas_flash_write_meta(struct isfdas_dev *idev,
								   const struct isfdas_flash_meta *meta_in)
{
	struct mtd_info *mtd;
	struct erase_info erase = {};
	struct isfdas_flash_meta meta = *meta_in;
	size_t retlen;
	loff_t addr;
	int ret;

	meta.crc = crc32(0, (const void *)&meta,
					 offsetof(struct isfdas_flash_meta, crc));

	mtd = get_mtd_device_nm("w25q64");
	if (IS_ERR(mtd))
		return PTR_ERR(mtd);

	/* switch to the other slot before writing */
	idev->flash_meta_slot ^= 1;
	addr = idev->flash_meta_slot ? ISFDAS_FLASH_META_SLOT1_OFFSET
								 : ISFDAS_FLASH_META_SLOT0_OFFSET;

	erase.addr = addr;
	erase.len = mtd->erasesize;
	ret = mtd_erase(mtd, &erase);
	if (ret)
		goto out;

	// ret = mtd_write(mtd, addr, sizeof(*meta), &retlen, (const u_char *)meta);
	ret = mtd_write(mtd, addr, sizeof(meta), &retlen, (const u_char *)&meta);
	if (ret)
		goto out;
	if (retlen != sizeof(meta))
		ret = -EIO;
out:
	put_mtd_device(mtd);
	return ret;
}
/*
 * isfdas_flash_recover() - rebuild our counters from flash at boot time
 *
 * This runs once when the module loads, before /dev/isfdas0 is
 * created, so nobody can submit a record while we are still figuring
 * out where we left off. If flash has no valid meta, we just start
 * fresh at seq 0.
 */
static void isfdas_flash_recover(struct isfdas_dev *idev)
{
	struct isfdas_flash_meta meta;
	int ret;

	ret = isfdas_flash_read_meta(idev, &meta); /* <-- only this line changed */
	// ret = isfdas_flash_read_meta(&meta);
	if (ret || meta.magic != ISFDAS_FLASH_META_MAGIC ||
		meta.version != ISFDAS_FLASH_META_VERSION)
	{
		pr_info("%s: no valid flash metadata (ret=%d), starting fresh\n",
				ISFDAS_MODULE_NAME, ret);
		atomic64_set(&idev->next_seq, 0);
		idev->flash_write_offset = 0;
		idev->flash_record_count = 0;
		return;
	}

	/* start one seq number after the last one we saved, so we never repeat */
	atomic64_set(&idev->next_seq, meta.last_seq + 1);
	idev->flash_write_offset = meta.write_offset;
	idev->flash_record_count = meta.record_count;

	pr_info("%s: recovered flash state: next_seq=%llu offset=%u count=%u\n",
			ISFDAS_MODULE_NAME,
			(unsigned long long)atomic64_read(&idev->next_seq),
			idev->flash_write_offset, idev->flash_record_count);
}

/*
 * isfdas_commit_record() - write one record to flash and update our counters
 *
 * All the bookkeeping (offset, count, seq) is only updated here in one
 * place, and only if the flash write actually succeeded.
 */

static int isfdas_commit_record(struct isfdas_dev *idev,
								struct sensor_record *rec)
{
	struct isfdas_flash_meta meta;
	int ret;

	/* step 1: write the actual record data to flash */
	ret = isfdas_flash_store(idev, rec);
	if (ret)
		return ret;

	mutex_lock(&idev->flash_meta_lock);

	/* step 2: move write pointer forward, wrap around at the end */
	idev->flash_write_offset = (idev->flash_write_offset +
								sizeof(*rec)) %
							   ISFDAS_FLASH_RECORD_AREA_SIZE;

	/* step 3: count how many records we have, don't go over max */
	if (idev->flash_record_count < (ISFDAS_FLASH_RECORD_AREA_SIZE / sizeof(*rec)))
		idev->flash_record_count++;

	idev->commits_since_meta++;

	mutex_unlock(&idev->flash_meta_lock);

	/* step 4: only save meta info every ISFDAS_META_BATCH records,
	 * not every single time, otherwise we erase flash way too often */
	if (idev->commits_since_meta < ISFDAS_META_BATCH)
		return 0;

	idev->commits_since_meta = 0;

	meta.magic = ISFDAS_FLASH_META_MAGIC;
	meta.version = ISFDAS_FLASH_META_VERSION;
	meta.last_seq = rec->seq;
	meta.write_offset = idev->flash_write_offset;
	meta.record_count = idev->flash_record_count;

	ret = isfdas_flash_write_meta(idev, &meta);
	if (ret)
		pr_warn("%s: metadata persist failed for seq=%llu: %d\n",
				ISFDAS_MODULE_NAME, (unsigned long long)rec->seq, ret);

	return 0;
}

/*
 * isfdas_work_handler() - workqueue function that drains flash_fifo
 *
 * Runs in normal process context, so it is fine to block here (flash
 * writes take time). Keeps draining until flash_fifo is empty, since
 * queue_work() can combine multiple pending calls into one run.
 * Only touches flash_fifo, never touches user_fifo.
 */
static void isfdas_work_handler(struct work_struct *work)
{
	struct isfdas_dev *idev = container_of(work, struct isfdas_dev,
										   flash_work);
	struct sensor_record rec;
	unsigned long flags;
	int ret;

	for (;;)
	{
		spin_lock_irqsave(&idev->flash_fifo_lock, flags);
		if (!kfifo_len(&idev->flash_fifo))
		{
			spin_unlock_irqrestore(&idev->flash_fifo_lock, flags);
			break;
		}
		ret = kfifo_out(&idev->flash_fifo, &rec, sizeof(rec));
		spin_unlock_irqrestore(&idev->flash_fifo_lock, flags);

		if (ret != sizeof(rec))
		{
			pr_warn("%s: short kfifo_out (%d) on flash fifo, aborting drain\n",
					ISFDAS_MODULE_NAME, ret);
			break;
		}

		ret = isfdas_commit_record(idev, &rec);
		if (ret)
			pr_warn("%s: commit of seq=%llu failed: %d\n",
					ISFDAS_MODULE_NAME,
					(unsigned long long)rec.seq, ret);
	}
}

/*
 * isfdas_submit_record() - called by the MPU6050 driver to give us a new
 * record.
 *
 * The caller must already set rec->timestamp_ns before calling this,
 * we don't touch that field. What we do here:
 *   1. give the record a seq number
 *   2. save it as the latest record (for quick ioctl read)
 *   3. push it into both fifos
 *   4. wake up anyone waiting on read(), and schedule the flash write
 *
 * Called from process context (threaded IRQ), so mutex_lock is fine.
 */
int isfdas_submit_record(struct sensor_record *rec)
{
	struct isfdas_dev *idev = isfdas_dev_ptr;
	unsigned long flags;
	unsigned int queued;
	u64 seq;

	if (!idev)
		return -ENODEV;
	if (!rec)
		return -EINVAL;

	/*
	 * seq number is always given here, not by whoever called us.
	 * this way flash recovery can always tell if any records are
	 * missing or duplicated, no matter which driver submitted them.
	 * this overwrites whatever was in rec->seq before.
	 */
	seq = atomic64_fetch_inc(&idev->next_seq);
	rec->seq = seq;

	mutex_lock(&idev->latest_lock);
	idev->latest_record = *rec;
	idev->has_record = true;
	mutex_unlock(&idev->latest_lock);

	/*
	 * we push the record into two separate fifos because userspace
	 * and the flash workqueue read at different speeds. if we used
	 * one shared fifo, a slow reader on one side would block the
	 * other side, and both would fight over the same kfifo_out calls.
	 */
	spin_lock_irqsave(&idev->user_fifo_lock, flags);
	queued = kfifo_in(&idev->user_fifo, rec, sizeof(*rec));
	spin_unlock_irqrestore(&idev->user_fifo_lock, flags);
	if (queued != sizeof(*rec))
		pr_warn_ratelimited("%s: user fifo full, seq=%llu dropped for userspace\n",
							ISFDAS_MODULE_NAME,
							(unsigned long long)seq);

	spin_lock_irqsave(&idev->flash_fifo_lock, flags);
	queued = kfifo_in(&idev->flash_fifo, rec, sizeof(*rec));
	spin_unlock_irqrestore(&idev->flash_fifo_lock, flags);
	if (queued != sizeof(*rec))
		pr_warn_ratelimited("%s: flash fifo full, seq=%llu dropped for persistence\n",
							ISFDAS_MODULE_NAME,
							(unsigned long long)seq);

	wake_up_interruptible(&idev->read_wq);

	/* queue_work is safe to call again even if work is already
	 * pending, so bursts of calls just collapse into one run */
	queue_work(idev->wq, &idev->flash_work);

	return 0;
}
EXPORT_SYMBOL_GPL(isfdas_submit_record);

/*
 * File operations for /dev/isfdas0
 */

/*
 * isfdas_open() - just checks the device exists and saves a pointer
 * to it. Nothing else to do yet, but kept here in case we need
 * per-open state later.
 */
static int isfdas_open(struct inode *inode, struct file *file)
{
	if (!isfdas_dev_ptr)
		return -ENODEV;

	file->private_data = isfdas_dev_ptr;
	return 0;
}

/*
 * isfdas_release() - matching cleanup for isfdas_open(). Nothing to
 * free right now, just clears the pointer.
 */
static int isfdas_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

/*
 * isfdas_read() - copies records from user_fifo out to userspace.
 *
 * Only copies whole records, never partial ones. If the userspace
 * buffer is too small to hold even one record, we return -EINVAL.
 * Blocks until data shows up unless O_NONBLOCK was set. Only touches
 * user_fifo, never flash_fifo.
 */
static ssize_t isfdas_read(struct file *file, char __user *buf,
						   size_t count, loff_t *ppos)
{
	struct isfdas_dev *idev = file->private_data;
	struct sensor_record rec;
	unsigned long flags;
	size_t copied = 0;
	int ret;

	if (count < sizeof(rec))
		return -EINVAL;

	while (copied + sizeof(rec) <= count)
	{
		spin_lock_irqsave(&idev->user_fifo_lock, flags);
		if (!kfifo_len(&idev->user_fifo))
		{
			spin_unlock_irqrestore(&idev->user_fifo_lock, flags);

			if (copied)
				break; /* give back what we already copied */

			if (file->f_flags & O_NONBLOCK)
				return -EAGAIN;

			ret = wait_event_interruptible(idev->read_wq,
										   kfifo_len(&idev->user_fifo) != 0);
			if (ret)
				return ret; /* -ERESTARTSYS */

			continue;
		}

		ret = kfifo_out(&idev->user_fifo, &rec, sizeof(rec));
		spin_unlock_irqrestore(&idev->user_fifo_lock, flags);

		if (ret != sizeof(rec))
			break;

		if (copy_to_user(buf + copied, &rec, sizeof(rec)))
			return -EFAULT;

		copied += sizeof(rec);
	}

	return copied;
}

/*
 * isfdas_poll() - lets userspace use select()/epoll() on this device
 * instead of only non-blocking read(). We only care about read
 * readiness here since this device has no write(). Based only on
 * user_fifo, never flash_fifo.
 */
static __poll_t isfdas_poll(struct file *file, struct poll_table_struct *wait)
{
	struct isfdas_dev *idev = file->private_data;
	__poll_t mask = 0;

	poll_wait(file, &idev->read_wq, wait);

	if (kfifo_len(&idev->user_fifo))
		mask |= EPOLLIN | EPOLLRDNORM;

	return mask;
}

/*
 * isfdas_ioctl() - handles our 3 ioctl commands
 *
 * GET_LATEST      -> gives back the RAM snapshot, doesn't touch either fifo
 * GET_FLASH_STATS -> gives back seq/offset/count numbers
 * FLUSH           -> forces flash_fifo to drain right now instead of
 *                     waiting for the workqueue to run on its own
 */
static long isfdas_ioctl(struct file *file, unsigned int cmd,
						 unsigned long arg)
{
	struct isfdas_dev *idev = file->private_data;
	struct sensor_record snapshot;
	struct isfdas_flash_stats stats;
	bool has_record;

	switch (cmd)
	{
	case ISFDAS_IOC_GET_LATEST:
		mutex_lock(&idev->latest_lock);
		snapshot = idev->latest_record;
		has_record = idev->has_record;
		mutex_unlock(&idev->latest_lock);

		if (!has_record)
			return -ENODATA;

		if (copy_to_user((void __user *)arg, &snapshot,
						 sizeof(snapshot)))
			return -EFAULT;
		return 0;

	case ISFDAS_IOC_GET_FLASH_STATS:
		mutex_lock(&idev->flash_meta_lock);
		stats.next_seq = atomic64_read(&idev->next_seq);
		stats.write_offset = idev->flash_write_offset;
		stats.record_count = idev->flash_record_count;
		mutex_unlock(&idev->flash_meta_lock);

		if (copy_to_user((void __user *)arg, &stats, sizeof(stats)))
			return -EFAULT;
		return 0;

	case ISFDAS_IOC_FLUSH:
		queue_work(idev->wq, &idev->flash_work);
		flush_work(&idev->flash_work);
		return 0;

	default:
		return -ENOTTY;
	}
}

static const struct file_operations isfdas_fops = {
	.owner = THIS_MODULE,
	.open = isfdas_open,
	.release = isfdas_release,
	.read = isfdas_read,
	.poll = isfdas_poll,
	.unlocked_ioctl = isfdas_ioctl,
	// .llseek		= no_llseek,
};

/*
 * Module init / exit
 *
 * ISFDAS doesn't sit on a bus by itself (it just connects two other
 * drivers together), so we use a plain module_init/module_exit here
 * instead of a platform_driver.
 */
static int __init isfdas_init(void)
{
	struct isfdas_dev *idev;
	struct device *dev_node;
	int ret;

	idev = kzalloc(sizeof(*idev), GFP_KERNEL);
	if (!idev)
		return -ENOMEM;

	mutex_init(&idev->latest_lock);
	mutex_init(&idev->flash_meta_lock);
	spin_lock_init(&idev->user_fifo_lock);
	spin_lock_init(&idev->flash_fifo_lock);
	init_waitqueue_head(&idev->read_wq);
	INIT_WORK(&idev->flash_work, isfdas_work_handler);

	/* recover our counters from flash before anything is made
	 * visible to userspace */
	isfdas_flash_recover(idev);

	{
		struct mtd_info *mtd_chk = get_mtd_device_nm("w25q64");
		if (!IS_ERR(mtd_chk))
		{
			if (mtd_chk->erasesize != W25Q64_SECTOR_SIZE)
				pr_warn("%s: erasesize mismatch: mtd=%u expected=%u\n",
						ISFDAS_MODULE_NAME, mtd_chk->erasesize,
						W25Q64_SECTOR_SIZE);
			put_mtd_device(mtd_chk);
		}
	}

	ret = kfifo_alloc(&idev->user_fifo,
					  ISFDAS_USER_FIFO_DEPTH * sizeof(struct sensor_record),
					  GFP_KERNEL);
	if (ret)
		goto err_free_dev;

	ret = kfifo_alloc(&idev->flash_fifo,
					  ISFDAS_FLASH_FIFO_DEPTH * sizeof(struct sensor_record),
					  GFP_KERNEL);
	if (ret)
		goto err_free_user_fifo;

	/* our own workqueue instead of the shared system one, so flash
	 * writes always happen in order and don't get delayed by other
	 * unrelated work */
	idev->wq = alloc_ordered_workqueue("%s", 0, ISFDAS_MODULE_NAME);
	if (!idev->wq)
	{
		ret = -ENOMEM;
		goto err_free_flash_fifo;
	}

	ret = alloc_chrdev_region(&idev->devt, 0, 1, ISFDAS_MODULE_NAME);
	if (ret)
		goto err_destroy_wq;
	idev->major = MAJOR(idev->devt);

	cdev_init(&idev->cdev, &isfdas_fops);
	idev->cdev.owner = THIS_MODULE;

	ret = cdev_add(&idev->cdev, idev->devt, 1);
	if (ret)
		goto err_unregister_chrdev;
	// idev->class = class_create(THIS_MODULE, ISFDAS_CLASS_NAME);
	idev->class = class_create(ISFDAS_CLASS_NAME);
	if (IS_ERR(idev->class))
	{
		ret = PTR_ERR(idev->class);
		goto err_del_cdev;
	}

	dev_node = device_create(idev->class, NULL, idev->devt, NULL,
							 ISFDAS_DEVICE_NAME);
	if (IS_ERR(dev_node))
	{
		ret = PTR_ERR(dev_node);
		goto err_destroy_class;
	}

	/* only set this pointer once everything above worked fine, so
	 * nobody sees a half-set-up device */
	isfdas_dev_ptr = idev;

	pr_info("%s: ready, /dev/%s major=%d next_seq=%llu\n",
			ISFDAS_MODULE_NAME, ISFDAS_DEVICE_NAME, idev->major,
			(unsigned long long)atomic64_read(&idev->next_seq));
	return 0;

err_destroy_class:
	class_destroy(idev->class);
err_del_cdev:
	cdev_del(&idev->cdev);
err_unregister_chrdev:
	unregister_chrdev_region(idev->devt, 1);
err_destroy_wq:
	destroy_workqueue(idev->wq);
err_free_flash_fifo:
	kfifo_free(&idev->flash_fifo);
err_free_user_fifo:
	kfifo_free(&idev->user_fifo);
err_free_dev:
	kfree(idev);
	return ret;
}

static void __exit isfdas_exit(void)
{
	struct isfdas_dev *idev = isfdas_dev_ptr;

	if (!idev)
		return;

	/* clear the pointer first so no new calls can come in while we
	 * are tearing things down */
	isfdas_dev_ptr = NULL;

	device_destroy(idev->class, idev->devt);
	class_destroy(idev->class);
	cdev_del(&idev->cdev);
	unregister_chrdev_region(idev->devt, 1);

	/* make sure the workqueue is fully stopped before we free the
	 * fifos it uses */
	destroy_workqueue(idev->wq);

	kfifo_free(&idev->flash_fifo);
	kfifo_free(&idev->user_fifo);
	kfree(idev);

	pr_info("%s: unloaded\n", ISFDAS_MODULE_NAME);
}

module_init(isfdas_init);
module_exit(isfdas_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Manjunatha V");
MODULE_DESCRIPTION("ISFDAS manager: coordinates MPU6050 (I2C) and W25Q64 (SPI/MTD) drivers");
MODULE_VERSION("2.0");