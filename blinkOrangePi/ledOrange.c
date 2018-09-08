
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include "ledOrange.h"

#define BUF_LEN 255
#define LEDPATH "/sys/class/leds/orangepi:red:status/trigger"
static struct file *f;



int writeCommand(unsigned char comm){
    ssize_t n = 0;
    loff_t offset = 0;
    mm_segment_t fs;

    char buff[ BUF_LEN + 1 ] = LEDPATH;

    f = filp_open( buff,O_CREAT | O_RDWR | O_TRUNC,S_IRUSR | S_IWUSR );
 
    if( IS_ERR( f ) ) {
    printk( "File open led failed: %s\n", buff );
    return -ENOENT;
    } 
    
    printk( "File open %s\n", buff);
    fs = get_fs();
    set_fs( get_ds() );

    switch (comm){
        case 0: 
                strcpy( buff, BLINK);
                break;
        case 1: 
                strcpy( buff, LEDON);
                break;            
        case 2: 
                strcpy( buff, LEDOFF);
                break;                      
            
        break;
        }    


    
    if( ( n = vfs_write( f, buff, strlen( buff ), &offset ) ) != strlen( buff ) ) {
    printk( "! failed to write: %d\n", n );
    return -EIO;
    }

    printk( "Blinl\n", n );
    set_fs( fs );
    filp_close( f, NULL );
    printk( "File close\n" );
    return -1;
}


static int __init ledStart( void ) {
    printk( "Blink module load!:\n");
    writeCommand(0);

    return 0;
}

static void __exit ledStop( void ) { 
   printk(KERN_NOTICE "Blink module stop\n" );
   writeCommand(2); 
}

module_init( ledStart ); 
module_exit( ledStop );
MODULE_LICENSE( "GPL" ); 