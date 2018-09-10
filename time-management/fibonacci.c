#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/pci.h>
#include <linux/version.h>
#include <linux/init.h>
#include <linux/timer.h>

static u64 a;
static u64 b = 1;
static struct timer_list fibonacci_timer = { .expires = 0 };

void next_fibonacci(struct timer_list *_)
{
	u64 c = a + b;

	a = b;
	b = c;

	fibonacci_timer.expires = get_jiffies_64() + HZ;
	add_timer(&fibonacci_timer);
}

static ssize_t fibonacci_show(struct class *class,
				struct class_attribute *attr,
				char *buf)
{
	//next_fibonacci(NULL);
	sprintf(buf, "%llu", a);
	return strlen(buf);
}

CLASS_ATTR_RO(fibonacci);

static struct class *fibonacci_class;

int __init init(void)
{
	int res;

	fibonacci_class = class_create(THIS_MODULE, "fibonacci-class");
	if (IS_ERR(fibonacci_class))
		pr_warn("bad class create\n");
	res = class_create_file(fibonacci_class, &class_attr_fibonacci);
	fibonacci_timer.expires = get_jiffies_64() + HZ;
	fibonacci_timer.function = next_fibonacci;
	add_timer(&fibonacci_timer);
	pr_info("'fibonacci' module initialized\n");
	return res;
}

void __exit cleanup(void)
{
	if (fibonacci_timer.expires != 0)
		del_timer(&fibonacci_timer);
	class_remove_file(fibonacci_class, &class_attr_fibonacci);
	class_destroy(fibonacci_class);
}

module_init(init);
module_exit(cleanup);
MODULE_LICENSE("GPL");
