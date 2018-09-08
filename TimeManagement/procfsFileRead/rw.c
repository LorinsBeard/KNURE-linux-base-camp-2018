/*
  Implement kernel module with API in sysfs or procfs, which returns time (in seconds) passed since previous read of it.
*/
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <linux/slab.h>
#include <linux/time.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Aleksandr Mordyk");
MODULE_DESCRIPTION("Home-work lesson 7");
MODULE_VERSION("0.1");

#define MODULE_TAG      "lesson7"
#define PROC_DIRECTORY  "lesson7"
#define PROC_FILENAME   "ls7"
#define BUFFER_SIZE     10


static char *proc_buffer;
static size_t proc_msg_length;
static size_t proc_msg_read_pos;

static struct proc_dir_entry *proc_dir;
static struct proc_dir_entry *proc_file;

static int example_read(struct file *file_p, char __user *buffer, size_t length, loff_t *offset);

static struct file_operations proc_fops = {
    .read  = example_read,
};


struct timeval tstart, tend;

static int create_buffer(void)
{
    proc_buffer = kmalloc(BUFFER_SIZE, GFP_KERNEL);
    if (NULL == proc_buffer)
        return -ENOMEM;
    proc_msg_length = 0;

    return 0;
}


static void cleanup_buffer(void)
{
    if (proc_buffer) {
        kfree(proc_buffer);
        proc_buffer = NULL;
    }
    proc_msg_length = 0;
}


static int create_proc_example(void)
{
    proc_dir = proc_mkdir(PROC_DIRECTORY, NULL);
    if (NULL == proc_dir)
        return -EFAULT;

    proc_file = proc_create(PROC_FILENAME, S_IFREG | S_IRUGO | S_IWUGO, proc_dir, &proc_fops);
    if (NULL == proc_file)
        return -EFAULT;

    return 0;
}

static void cleanup_proc_example(void)
{
    if (proc_file)
    {
        remove_proc_entry(PROC_FILENAME, proc_dir);
        proc_file = NULL;
    }
    if (proc_dir)
    {
        remove_proc_entry(PROC_DIRECTORY, NULL);
        proc_dir = NULL;
    }
}


static int example_read(struct file *file_p, char __user *buffer, size_t length, loff_t *offset)
{
    size_t left;

    if (length > (proc_msg_length - proc_msg_read_pos))
        length = (proc_msg_length - proc_msg_read_pos);

    left = raw_copy_to_user(buffer, &proc_buffer[proc_msg_read_pos], length);

    proc_msg_read_pos += length - left;

    if (left)
        printk(KERN_ERR MODULE_TAG "failed to read %u from %u chars\n", left, length);
    else {
      
	do_gettimeofday(&tend);
	printk("Time since past treatment: %ld sec\n", (tend.tv_sec - tstart.tv_sec));
	do_gettimeofday(&tstart);
	}
    return length - left;
}

static int __init example_init(void)
{
    int err;
	do_gettimeofday(&tstart); //start timer

    err = create_buffer();
    if (err)
        goto error;

    err = create_proc_example();
    if (err)
        goto error;

    printk(KERN_NOTICE MODULE_TAG "-loaded\n");
    return 0;

error:
    printk(KERN_ERR MODULE_TAG "failed to load\n");
    cleanup_proc_example();
    cleanup_buffer();
    return err;
}


static void __exit example_exit(void)
{
    cleanup_proc_example();
    cleanup_buffer();
    printk(KERN_NOTICE MODULE_TAG "exited\n");
}


module_init(example_init);
module_exit(example_exit);
