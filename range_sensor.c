#include <linux/version.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/interrupt.h> 

#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/amlogic/gpio-amlogic.h>
#include <linux/wait.h>
#include <linux/sched.h>

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Sergio Tanzilli, OtherCrashOverride, Winston Todd");
MODULE_DESCRIPTION("Driver for RadioShack ultrasonic sensor (sku 2760342)");


// TODO: Refactor to allow multiple instances for using more
//       than once sensor.  This is complicated by the Amlogic
//       GPIO IRQ limit of (8) entries.  Rising and falling
//       each require a hardware entry thus limiting it to 
//       four (8 / 2 = 4).

typedef enum range_sensor_status
{
	RANGE_SENSOR_STATUS_READY = 0,
	RANGE_SENSOR_STATUS_WAITING_FOR_RESPONSE_START,
	RANGE_SENSOR_STATUS_WAITING_FOR_RESPONSE_END,
	RANGE_SENSOR_STATUS_COMPLETE
} range_sensor_status_t;


static int rising_irq = -1;
static int falling_irq = -1;

volatile static range_sensor_status_t status = RANGE_SENSOR_STATUS_READY;
volatile static ktime_t response_start;
volatile static ktime_t response_end;

DECLARE_WAIT_QUEUE_HEAD(wq);

static const char* GPIO_OWNER = "range_sensor";

static int signal_gpio = 102;
module_param(signal_gpio, int, S_IRUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(signal_gpio, "Signal GPIO pin number");


// This function is called when you write something on /sys/class/range_sensor/value
static ssize_t range_sensor_value_write(struct class *class, struct class_attribute *attr, const char *buf, size_t len)
{
	// Currently unused
	return len;
}

// This function is called when you read /sys/class/range_sensor/value
// Note: https://www.kernel.org/doc/Documentation/filesystems/sysfs.txt
//		 "sysfs allocates a buffer of size (PAGE_SIZE) and passes it to the
//		 method."
static ssize_t range_sensor_value_read(struct class *class, struct class_attribute *attr, char *buf)
{
	int wait_result;
	int return_value;
	int rtc;

	// Set the signal gpio direction to output so we can send a pulse
	rtc = amlogic_gpio_direction_output(signal_gpio, 0, GPIO_OWNER);
	if (rtc != 0)
	{
		printk(KERN_ERR "Signal GPIO direction set failed.");
		goto fail;
	}

	// Pulse the signal line to initiate a range scan
	udelay(2);
	amlogic_set_value(signal_gpio, 1, GPIO_OWNER);
	udelay(5);
	amlogic_set_value(signal_gpio, 0, GPIO_OWNER);

	// Set the status to RANGE_SENSOR_STATUS_WAITING_FOR_RESPONSE_START,
	// so the irq handler knows to interpret the next rising edge as the
	// beginning of the response pulse
	status = RANGE_SENSOR_STATUS_WAITING_FOR_RESPONSE_START;

	// Set the signal gpio as input so we can receive the response
	rtc = amlogic_gpio_direction_input(signal_gpio, GPIO_OWNER);
	if (rtc != 0)
	{
		printk(KERN_ERR "Signal GPIO direction set failed.");
		goto fail;
	}

	// Wait up to 32ms for a response
	wait_result = wait_event_timeout(wq, status == RANGE_SENSOR_STATUS_COMPLETE, 32 * HZ / 1000);
	status = RANGE_SENSOR_STATUS_READY;

	if (wait_result == 0)
	{	// Timeout occured
		return_value = sprintf(buf, "%d\n", -1);
	}
	else
	{
		return_value = sprintf(buf, "%lld\n", ktime_to_us(ktime_sub(response_end, response_start)));
	}

	return return_value;

fail:
	status = RANGE_SENSOR_STATUS_READY;
	return sprintf(buf, "%d\n", -1);
}

// Sysfs definitions for range_sensor class
static struct class_attribute range_sensor_class_attrs[] =
{
	__ATTR(value, S_IRUGO | S_IWUSR, range_sensor_value_read, range_sensor_value_write),
	__ATTR_NULL,
};

// Name of directory created in /sys/class
static struct class range_sensor_class =
{
	.name = "range_sensor",
	.owner = THIS_MODULE,
	.class_attrs = range_sensor_class_attrs,
};

// Interrupt handler on signal_gpio
static irqreturn_t gpio_isr_rising(int irq, void *data)
{
	// Rising edge
	if (status == RANGE_SENSOR_STATUS_WAITING_FOR_RESPONSE_START)
	{
		response_start = ktime_get();
		response_end = response_start;

		status = RANGE_SENSOR_STATUS_WAITING_FOR_RESPONSE_END;

	}

	return IRQ_HANDLED;
}

// Interrupt handler on signal_gpio
static irqreturn_t gpio_isr_falling(int irq, void *data)
{
	// Falling edge
	if (status == RANGE_SENSOR_STATUS_WAITING_FOR_RESPONSE_END)
	{
		response_end = ktime_get();
		status = RANGE_SENSOR_STATUS_COMPLETE;

		wake_up(&wq);
	}

	return IRQ_HANDLED;
}

static int range_sensor_init(void)
{
	int rtc;
	int irq_banks[2];

	printk(KERN_INFO "RadioShack Ultrasonic Range Sensor driver initializing.\n");
	printk(KERN_INFO "Signal GPIO: %d.\n", signal_gpio);
	if (class_register(&range_sensor_class) < 0)
	{
		goto fail_1;
	}


	// Setup signal gpio
	rtc = amlogic_gpio_request(signal_gpio, GPIO_OWNER);
	if (rtc != 0)
	{
		printk(KERN_ERR "Switch GPIO request failed.\n");
		goto fail_2;
	}

	rtc = amlogic_gpio_direction_input(signal_gpio, GPIO_OWNER);
	if (rtc != 0)
	{
		printk(KERN_ERR "Signal GPIO direction set failed.");
		goto fail_2;
	}

	amlogic_disable_pullup(signal_gpio, GPIO_OWNER);


	// Set RISING irq
	rising_irq = (GPIO_IRQ0 + INT_GPIO_0);
	rtc = amlogic_gpio_to_irq(signal_gpio, GPIO_OWNER, AML_GPIO_IRQ(rising_irq, FILTER_NUM1, GPIO_IRQ_RISING));
	if (rtc < 0)
	{
		printk(KERN_ERR "Rising IRQ mapping failed.\n");
		goto fail_2;
	}

	rtc = request_irq(rising_irq, (irq_handler_t)gpio_isr_rising, IRQF_DISABLED, "range_sensor", NULL);
	if (rtc)
	{
		printk(KERN_ERR "Rising IRQ:%d request failed. (Error=%d)\n", rising_irq, rtc);
		goto fail_2;
	}
	else
	{
		printk(KERN_ERR "Rising IRQ:%d\n", rising_irq);
	}


	// Set FALLING irq
	falling_irq = (GPIO_IRQ1 + INT_GPIO_0);
	rtc = amlogic_gpio_to_irq(signal_gpio, GPIO_OWNER, AML_GPIO_IRQ(falling_irq, FILTER_NUM1, GPIO_IRQ_FALLING));
	if (rtc < 0)
	{
		printk(KERN_ERR "Falling IRQ mapping failed.\n");
		goto fail_2;
	}

	rtc = request_irq(falling_irq, (irq_handler_t)gpio_isr_falling, IRQF_DISABLED, "range_sensor", NULL);
	if (rtc)
	{
		printk(KERN_ERR "Falling IRQ:%d request failed. (Error=%d)\n", falling_irq, rtc);
		goto fail_3;
	}
	else
	{
		printk(KERN_ERR "Falling IRQ:%d\n", falling_irq);
	}


	printk(KERN_INFO "RadioShack Ultrasonic Range Sensor driver installed.\n");
	return 0;

fail_3:
	meson_free_irq(signal_gpio, irq_banks);

	if (rising_irq != -1)
	{
		free_irq(rising_irq, NULL);
	}

fail_2:
	amlogic_gpio_free(signal_gpio, GPIO_OWNER);

	class_unregister(&range_sensor_class);

fail_1:
	return -1;

}

static void range_sensor_exit(void)
{
	int irq_banks[2];

	disable_irq(falling_irq);
	disable_irq(rising_irq);

	meson_free_irq(signal_gpio, irq_banks);

	if (rising_irq != -1)
		free_irq(rising_irq, NULL);

	if (falling_irq != -1)
		free_irq(falling_irq, NULL);

	amlogic_gpio_free(signal_gpio, GPIO_OWNER);

	class_unregister(&range_sensor_class);

	printk(KERN_INFO "RadioShack Ultrasonic Range Sensor driver removed.\n");
}

module_init(range_sensor_init);
module_exit(range_sensor_exit);
