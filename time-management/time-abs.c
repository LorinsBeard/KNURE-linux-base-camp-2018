#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/pci.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/time.h>

static struct timespec64 last_read_abs_time;

static ssize_t last_access_absolute_time_show( struct class *class, struct class_attribute *attr, char *buf ) {
    if (last_read_abs_time.tv_sec == 0) {
        sprintf(buf, "n/a");
    } else {
        struct tm parsed_datetime;
        time_to_tm(last_read_abs_time.tv_sec, 0, &parsed_datetime);
        sprintf(
            buf,
            "%ld/%02d/%02d %02d:%02d:%02d.%09ld",
            parsed_datetime.tm_year + 1900,
            parsed_datetime.tm_mon + 1,
            parsed_datetime.tm_mday,
            parsed_datetime.tm_hour,
            parsed_datetime.tm_min, 
            parsed_datetime.tm_sec,
            last_read_abs_time.tv_nsec
        );
    }
    last_read_abs_time = current_kernel_time64();
    return strlen( buf );
}

CLASS_ATTR_RO( last_access_absolute_time );

static struct class *time_class;

int __init init(void) {
    int res;
    last_read_abs_time.tv_sec = 0;
    time_class = class_create( THIS_MODULE, "time-class" );
    if( IS_ERR( time_class ) ) printk( "bad class create\n" );
    res = class_create_file( time_class, &class_attr_last_access_absolute_time );
    printk( "'time-abs' module initialized\n" );
    return res;
}

void __exit cleanup(void) {
    class_remove_file( time_class, &class_attr_last_access_absolute_time );
    class_destroy( time_class );
    return;
}

module_init( init );
module_exit( cleanup );
MODULE_LICENSE( "GPL" );
