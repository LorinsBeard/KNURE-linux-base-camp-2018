#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/time.h>

static u64 last_read_jiffies;

static ssize_t last_access_time_diff_show( struct class *class, struct class_attribute *attr, char *buf ) {
    u64 old_last_read_jiffies = last_read_jiffies;
    last_read_jiffies = get_jiffies_64();
    if (old_last_read_jiffies == 0) {
        sprintf(buf, "0");
    } else {
        struct timespec ts;
        jiffies_to_timespec((last_read_jiffies - old_last_read_jiffies), &ts);
        sprintf(buf, "%lu",  ts.tv_sec);
    }
    return strlen( buf );
}

CLASS_ATTR_RO( last_access_time_diff );

static struct class *time_class;

int __init init(void) {
    int res;
    last_read_jiffies = 0;
    time_class = class_create( THIS_MODULE, "x-class" );
    if( IS_ERR( time_class ) ) printk( "bad class create\n" );
    res = class_create_file( time_class, &class_attr_last_access_time_diff );
    printk( "'time-diff' module initialized\n" );
    return res;
}

void __exit cleanup(void) {
    class_remove_file( time_class, &class_attr_last_access_time_diff );
    class_destroy( time_class );
    return;
}

module_init( init );
module_exit( cleanup );
MODULE_LICENSE( "GPL" );
