/*
 * Microchip Serial Touchscreen Driver
 *
 * Copyright (c) 2010 Microchip Technology, Inc.
 * 
 * http://www.microchip.com/tsharc
 */

/*
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

/*
 * This driver can handle serial Microchip controllers using the TSHARC 4-byte protocol
 */

#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/serio.h>
#include "serio-mchp.h"
#include <linux/init.h>
#include <linux/ctype.h>

#define DRIVER_DESC	"Microchip TSHARC Serial Touchscreen Driver"

MODULE_AUTHOR("Steve Grahovac <steve.grahovac@microchip.com>");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*
 * Definitions & global arrays.
 */

#define TSHARC_MAX_LENGTH	4

/*
 * Per-touchscreen controller data.
 */

struct tsharc {
	struct input_dev *dev;
	struct serio *serio;
	int id;
	int index;
	unsigned char data[TSHARC_MAX_LENGTH];
	char phys[32];
};

static void tsharc_process_data(struct tsharc *tsharc, unsigned char data)
{
	struct input_dev *dev = tsharc->dev;
	int x,y,touch;
	
	tsharc->data[tsharc->index] = data;

	/****************************************************
	
	Data format, 4 bytes: SYNC, DATA1, DATA2, DATA3
	
	SYNC [7:0]: 1,TOUCH,X[11:9],Y[11:9]
	DATA1[7:0]: 0,X[8:2]
	DATA2[7:0]: 0,Y[8:2]
	DATA3[7:0]: 0,0,0,0,X[1:0],Y[1:0]
	
	TOUCH: 0 = Touch up, 1 = Touch down
	
	****************************************************/	
	
	switch (tsharc->index++) {
		case 0:
			if (!(0x80 & data))
			{
/*			    printk(KERN_DEBUG "tsharc: Sync bit not set\n");	  */
			    tsharc->index = 0;			  
			}
			break;

		case (TSHARC_MAX_LENGTH - 1):
			tsharc->index = 0;
	
			/* verify byte is valid for current index*/
			if (0x80 & data)
			{
				/* byte not valid */
				tsharc->data[0]=data;
				tsharc->index = 1;
				break;
			}


			
			x = ((tsharc->data[0] & 0x38) << 6) | ((tsharc->data[1] & 0x7f) << 2) | ((tsharc->data[3] & 0x0c) >> 2);
			y = ((tsharc->data[0] & 0x07) << 9) | ((tsharc->data[2] & 0x7f) << 2) | ((tsharc->data[3] & 0x03));
			touch = (tsharc->data[0] & 0x40) >> 6;
			
/*			printk(KERN_DEBUG "tsharc: x=%d, y=%d, touch=%d (raw bytes: 0x%x 0x%x 0x%x 0x%x)\n", x,y,touch,tsharc->data[0], tsharc->data[1], tsharc->data[2] , tsharc->data[3]); */

			input_report_abs(dev, ABS_X, x);
			input_report_abs(dev, ABS_Y, y);
			input_report_abs(dev, ABS_PRESSURE, touch * 255);
			input_report_key(dev, BTN_TOUCH, touch);
			
			input_sync(dev);			
			break;
		default:
			/* verify byte is valid for current index */
			if (0x80 & data)
			{
				/* byte not valid */			  
				tsharc->data[0]=data;
				tsharc->index = 1;
			}

			break;
	}
}


static irqreturn_t tsharc_interrupt(struct serio *serio,
		unsigned char data, unsigned int flags)
{
    struct tsharc *tsharc = serio_get_drvdata(serio);
    tsharc_process_data(tsharc, data);

    return IRQ_HANDLED;
}

static int tsharc_setup(struct tsharc *tsharc)
{
	struct input_dev *dev = tsharc->dev;

	input_set_abs_params(dev, ABS_X, 0, 4096, 0, 0);
	input_set_abs_params(dev, ABS_Y, 0, 4096, 0, 0);
	input_set_abs_params(dev, ABS_PRESSURE, 0, 256, 0, 0);
	return 0;
}

/*
 * tsharc_disconnect() is the opposite of tsharc_connect()
 */

static void tsharc_disconnect(struct serio *serio)
{
	struct tsharc *tsharc = serio_get_drvdata(serio);

	input_get_device(tsharc->dev);
	input_unregister_device(tsharc->dev);
	serio_close(serio);
	serio_set_drvdata(serio, NULL);
	input_put_device(tsharc->dev);
	kfree(tsharc);
}

/*
 * tsharc_connect() is the routine that is called when someone adds a
 * new serio device that supports Tsharc protocol and registers it as
 * an input device.
 */

static int tsharc_connect(struct serio *serio, struct serio_driver *drv)
{
	struct tsharc *tsharc;
	struct input_dev *input_dev;
	int err;

	tsharc = kzalloc(sizeof(struct tsharc), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!tsharc || !input_dev) {
		err = -ENOMEM;
		goto fail1;
	}

	tsharc->serio 	= serio;
	tsharc->id  	= serio->id.id;
	tsharc->dev 	= input_dev;
	snprintf(tsharc->phys, sizeof(tsharc->phys), "%s/input0", serio->phys);

	input_dev->name 		= "Microchip TSHARC Serial TouchScreen";
	input_dev->phys 		= tsharc->phys;
	input_dev->id.bustype 	= BUS_RS232;
	input_dev->id.vendor 	= SERIO_MCHPTSHARC;
	input_dev->id.product 	= tsharc->id;
	input_dev->id.version 	= 0x0100;
	input_dev->dev.parent 	= &serio->dev;

	input_dev->evbit[0] 	= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	serio_set_drvdata(serio, tsharc);

	err = serio_open(serio, drv);
	if (err)
		goto fail2;


	tsharc_setup(tsharc);

	err = input_register_device(tsharc->dev);
	if (err)
		goto fail3;

	return 0;

 fail3: serio_close(serio);
 fail2:	serio_set_drvdata(serio, NULL);
 fail1:	input_free_device(input_dev);
	kfree(tsharc);
	return err;
}

/*
 * The serio driver structure.
 */

static struct serio_device_id tsharc_serio_ids[] = {
	{
		.type	= SERIO_RS232,
		.proto	= SERIO_MCHPTSHARC,
		.id		= SERIO_ANY,
		.extra	= SERIO_ANY,
	},
	{ 0 }
};

MODULE_DEVICE_TABLE(serio, tsharc_serio_ids);

static struct serio_driver tsharc_drv = {
	.driver		= {
		.name	= "tsharc",
	},
	.description	= DRIVER_DESC,
	.id_table		= tsharc_serio_ids,
	.interrupt		= tsharc_interrupt,
	.connect		= tsharc_connect,
	.disconnect		= tsharc_disconnect,
};

/*
 * The functions for inserting/removing us as a module.
 */

static int __init tsharc_init(void)
{
	return serio_register_driver(&tsharc_drv);
}

static void __exit tsharc_exit(void)
{
	serio_unregister_driver(&tsharc_drv);
}

module_init(tsharc_init);
module_exit(tsharc_exit);
