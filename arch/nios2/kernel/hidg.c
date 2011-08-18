#include <linux/platform_device.h>
#include <linux/usb/g_hid.h>

/* hid descriptor for a keyboard */
static struct hidg_func_descriptor my_keyboard_hid_data = {
	.subclass		= 0, /* No subclass */
	.protocol		= 1, /* Keyboard */
	.report_length		= 8,
	.report_desc_length	= 63,
	.report_desc		= {
		0x05, 0x01,	/* USAGE_PAGE (Generic Desktop)	          */
		0x09, 0x06,	/* USAGE (Keyboard)                       */
		0xa1, 0x01,	/* COLLECTION (Application)               */
		0x05, 0x07,	/*   USAGE_PAGE (Key Codes)               */
		0x19, 0xe0,	/*   USAGE_MINIMUM (Keyboard LeftControl) */
		0x29, 0xe7,	/*   USAGE_MAXIMUM (Keyboard Right GUI)   */
		0x15, 0x00,	/*   LOGICAL_MINIMUM (0)                  */
		0x25, 0x01,	/*   LOGICAL_MAXIMUM (1)                  */
		0x75, 0x01,	/*   REPORT_SIZE (1)                      */
		0x95, 0x08,	/*   REPORT_COUNT (8)                     */
		0x81, 0x02,	/*   INPUT (Data,Var,Abs)                 */
		0x95, 0x01,	/*   REPORT_COUNT (1)                     */
		0x75, 0x08,	/*   REPORT_SIZE (8)                      */
		0x81, 0x03,	/*   INPUT (Cnst,Var,Abs)                 */
		0x95, 0x05,	/*   REPORT_COUNT (5)                     */
		0x75, 0x01,	/*   REPORT_SIZE (1)                      */
		0x05, 0x08,	/*   USAGE_PAGE (LEDs)                    */
		0x19, 0x01,	/*   USAGE_MINIMUM (Num Lock)             */
		0x29, 0x05,	/*   USAGE_MAXIMUM (Kana)                 */
		0x91, 0x02,	/*   OUTPUT (Data,Var,Abs)                */
		0x95, 0x01,	/*   REPORT_COUNT (1)                     */
		0x75, 0x03,	/*   REPORT_SIZE (3)                      */
		0x91, 0x03,	/*   OUTPUT (Cnst,Var,Abs)                */
		0x95, 0x06,	/*   REPORT_COUNT (6)                     */
		0x75, 0x08,	/*   REPORT_SIZE (8)                      */
		0x15, 0x00,	/*   LOGICAL_MINIMUM (0)                  */
		0x25, 0x65,	/*   LOGICAL_MAXIMUM (101)                */
		0x05, 0x07,	/*   USAGE_PAGE (Keyboard)                */
		0x19, 0x00,	/*   USAGE_MINIMUM (Reserved)             */
		0x29, 0x65,	/*   USAGE_MAXIMUM (Keyboard Application) */
		0x81, 0x00,	/*   INPUT (Data,Ary,Abs)                 */
		0xc0		/* END_COLLECTION                         */
	}
};

/* hid descriptor for a mouse */
static struct hidg_func_descriptor my_mouse_hid_data = {
	.subclass		= 0, /* No subclass */
	.protocol		= 2, /* Mouse */
	.report_length		= 3,
	.report_desc_length	= 50,
	/* USB Device Class Definition for HID: E.10 Report Descriptor (Mouse) */
	.report_desc		= {
		0x05, 0x01,	/* USAGE_PAGE (Generic Desktop)	              */
		0x09, 0x02,	/* USAGE (Mouse)                              */
		0xa1, 0x01,	/* COLLECTION (Application)                   */
		0x09, 0x01,	/*   USAGE (Pointer)                          */
		0xa1, 0x00,	/*   COLLECTION (Physical)                    */
		0x05, 0x09,	/*     USAGE_PAGE (Buttons)                   */
		0x19, 0x01,	/*     USAGE_MINIMUM (01)                     */
		0x29, 0x03,	/*     USAGE_MAXIMUM (03)                     */
		0x15, 0x00,	/*     LOGICAL_MINIMUM (0)                    */
		0x25, 0x01,	/*     LOGICAL_MAXIMUM (1)                    */
		0x95, 0x03,	/*     REPORT_COUNT (3)                       */
		0x75, 0x01,	/*     REPORT_SIZE (1)                        */
		0x81, 0x02,	/*     INPUT (Data,Var,Abs)  ; 3 button bits  */
		0x95, 0x01,	/*     REPORT_COUNT (1)                       */
		0x75, 0x05,	/*     REPORT_SIZE (5)                        */
		0x81, 0x01,	/*     INPUT (Const)         ; 5 bits padding */
		0x05, 0x01,	/*     USAGE_PAGE (Generic Desktop)           */
		0x09, 0x30,	/*     USAGE (X)                              */
		0x09, 0x31,	/*     USAGE (Y)                              */
		0x15, 0x81,	/*     LOGICAL_MINIMUM (-127)                 */
		0x25, 0x7F,	/*     LOGICAL_MAXIMUM (127)                  */
		0x75, 0x08,	/*     REPORT_SIZE (8)                        */
		0x95, 0x02,	/*     REPORT_COUNT (2)                       */
		0x81, 0x06,	/*     INPUT (Data,Var,Rel)  ; 2 position bytes (X & Y) */
		0xc0,		/*   END_COLLECTION                           */
		0xc0		/* END_COLLECTION                             */
	}
};

static struct platform_device my_keyboard_hid = {
	.name			= "hidg",
	.id			= 0,
	.num_resources		= 0,
	.resource		= NULL,
	.dev.platform_data	= &my_keyboard_hid_data,
};

static struct platform_device my_mouse_hid = {
	.name			= "hidg",
	.id			= 1,
	.num_resources		= 0,
	.resource		= 0,
	.dev.platform_data	= &my_mouse_hid_data,
};

static int __init hid_gadget_init(void)
{
	int ret;

	ret = platform_device_register(&my_keyboard_hid);
	if (ret) {
		pr_info("can't register keyboard hid gadget device: %d\n", ret);
		return ret;
	}

	ret = platform_device_register(&my_mouse_hid);
	if (ret) {
		pr_info("can't register mouse hid gadget device: %d\n", ret);
		platform_device_unregister(&my_keyboard_hid);
		return ret;
	}

	return 0;
}
subsys_initcall(hid_gadget_init);

