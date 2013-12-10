/*
 * Copyright (C) 2012-2013 MundoReader S.L.
 * Author: Heiko Stuebner <heiko@sntech.de>
 *
 * based in parts on Nook zforce driver
 *
 * Copyright (C) 2010 Barnes & Noble, Inc.
 * Author: Pieter Truter<ptruter@intrinsyc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/input/zforce_ts.h>
#include <linux/input/mt.h>

#define WAIT_TIMEOUT		msecs_to_jiffies(1000)

#define FRAME_START		0xee

/* Offsets of the different parts of the payload the controller sends */
#define PAYLOAD_HEADER		0
#define PAYLOAD_LENGTH		1
#define PAYLOAD_BODY		2

/* Response offsets */
#define RESPONSE_ID		0
#define RESPONSE_DATA		1

/* Commands */
#define COMMAND_DEACTIVATE	0x00
#define COMMAND_INITIALIZE	0x01
#define COMMAND_RESOLUTION	0x02
#define COMMAND_SETCONFIG	0x03
#define COMMAND_DATAREQUEST	0x04
#define COMMAND_SCANFREQ	0x08
#define COMMAND_STATUS		0X1e

/*
 * Responses the controller sends as a result of
 * command requests
 */
#define RESPONSE_DEACTIVATE	0x00
#define RESPONSE_INITIALIZE	0x01
#define RESPONSE_RESOLUTION	0x02
#define RESPONSE_SETCONFIG	0x03
#define RESPONSE_SCANFREQ	0x08
#define RESPONSE_STATUS		0X1e

/*
 * Notifications are send by the touch controller without
 * beeing requested by the driver and include for example
 * touch indications
 */
#define NOTIFICATION_TOUCH		0x04
#define NOTIFICATION_BOOTCOMPLETE	0x07
#define NOTIFICATION_OVERRUN		0x25
#define NOTIFICATION_PROXIMITY		0x26
#define NOTIFICATION_INVALID_COMMAND	0xfe

#define ZFORCE_REPORT_POINTS		2
#define ZFORCE_MAX_AREA			0xff

#define STATE_DOWN			0
#define STATE_MOVE			1
#define STATE_UP			2

#define SETCONFIG_DUALTOUCH		(1 << 0)

struct zforce_point {
	int coord_x;
	int coord_y;
	int state;
	int id;
	int area_major;
	int area_minor;
	int orientation;
	int pressure;
	int prblty;
};

/*
 * @client		the i2c_client
 * @input		the input device
 * @suspending		in the process of going to suspend (don't emit wakeup
 *			events for commands executed to suspend the device)
 * @suspended		device suspended
 * @access_mutex	serialize i2c-access, to keep multipart reads together
 * @command_done	completion to wait for the command result
 * @command_mutex	serialize commands send to the ic
 * @command_waiting	the id of the command that that is currently waiting
 *			for a result
 * @command_result	returned result of the command
 */
struct zforce_ts {
	struct i2c_client	*client;
	struct input_dev	*input;
	char			phys[32];

	bool			stopped; /* FIXME: not for upstream */
	bool			suspending;
	bool			suspended;
	bool			boot_complete;

	/* Firmware version information */
	u16			version_major;
	u16			version_minor;
	u16			version_build;
	u16			version_rev;

	struct mutex		access_mutex;

	struct completion	command_done;
	struct mutex		command_mutex;
	int			command_waiting;
	int			command_result;

	/* FIXME: not for upstream */
	struct delayed_work	check;
	struct delayed_work	reset;
};

static int zforce_command(struct zforce_ts *ts, u8 cmd)
{
	struct i2c_client *client = ts->client;
	char buf[3];
	int ret;

	dev_dbg(&client->dev, "%s: 0x%x\n", __FUNCTION__, cmd);

	buf[0] = FRAME_START;
	buf[1] = 1; /* data size, command only */
	buf[2] = cmd;

	mutex_lock(&ts->access_mutex);
	ret = i2c_master_send(client, &buf[0], ARRAY_SIZE(buf));
	mutex_unlock(&ts->access_mutex);
	if (ret < 0) {
		dev_err(&client->dev, "i2c send data request error: %d\n", ret);
		return ret;
	}

	return 0;
}

static int zforce_send_wait(struct zforce_ts *ts, const char *buf, const int len)
{
	struct i2c_client *client = ts->client;
	int ret;

	ret = mutex_trylock(&ts->command_mutex);
	if (!ret) {
		dev_err(&client->dev, "already waiting for a command\n");
		return -EBUSY;
	}

	dev_dbg(&client->dev, "sending %d bytes for command 0x%x\n", buf[1], buf[2]);

	ts->command_waiting = buf[2];

	mutex_lock(&ts->access_mutex);
	ret = i2c_master_send(client, buf, len);
	mutex_unlock(&ts->access_mutex);
	if (ret < 0) {
		dev_err(&client->dev, "i2c send data request error: %d\n", ret);
		goto unlock;
	}

	dev_dbg(&client->dev, "waiting for result for command 0x%x\n", buf[2]);

	if (wait_for_completion_timeout(&ts->command_done, WAIT_TIMEOUT) == 0) {
		ret = -ETIME;
		goto unlock;
	}

	ret = ts->command_result;

unlock:
	mutex_unlock(&ts->command_mutex);
	return ret;
}

static int zforce_command_wait(struct zforce_ts *ts, u8 cmd)
{
	struct i2c_client *client = ts->client;
	char buf[3];
	int ret;

	dev_dbg(&client->dev, "%s: 0x%x\n", __FUNCTION__, cmd);

	buf[0] = FRAME_START;
	buf[1] = 1; /* data size, command only */
	buf[2] = cmd;

	ret = zforce_send_wait(ts, &buf[0], ARRAY_SIZE(buf));
	if (ret < 0) {
		dev_err(&client->dev, "i2c send data request error: %d\n", ret);
		return ret;
	}

	return 0;
}

static int zforce_resolution(struct zforce_ts *ts, u16 x, u16 y)
{
	struct i2c_client *client = ts->client;
	char buf[7] = { FRAME_START, 5, COMMAND_RESOLUTION,
			(x & 0xff), ((x >> 8) & 0xff),
			(y & 0xff), ((y >> 8) & 0xff) };

	dev_dbg(&client->dev, "set resolution to (%d,%d)\n", x, y);

	return zforce_send_wait(ts, &buf[0], ARRAY_SIZE(buf));
}

static int zforce_scan_frequency(struct zforce_ts *ts, u16 idle, u16 finger, u16 stylus)
{
	struct i2c_client *client = ts->client;
	char buf[9] = { FRAME_START, 7, COMMAND_SCANFREQ,
			(idle & 0xff), ((idle >> 8) & 0xff),
			(finger & 0xff), ((finger >> 8) & 0xff),
			(stylus & 0xff), ((stylus >> 8) & 0xff) };

	dev_dbg(&client->dev, "set scan frequency to (idle: %d, finger: %d, stylus: %d)\n",
		idle, finger, stylus);

	return zforce_send_wait(ts, &buf[0], ARRAY_SIZE(buf));
}

static int zforce_setconfig(struct zforce_ts *ts, char b1)
{
	struct i2c_client *client = ts->client;
	char buf[7] = { FRAME_START, 5, COMMAND_SETCONFIG,
			b1, 0, 0, 0 };

	dev_dbg(&client->dev, "set config to (%d)\n", b1);

	return zforce_send_wait(ts, &buf[0], ARRAY_SIZE(buf));
}

static int zforce_start(struct zforce_ts *ts)
{
	struct i2c_client *client = ts->client;
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;
	int ret;

	dev_dbg(&client->dev, "starting device\n");

	/* FIXME: not for upstream */
	ts->stopped = false;

	ret = zforce_command_wait(ts, COMMAND_INITIALIZE);
	if (ret) {
		dev_err(&client->dev, "Unable to initialize, %d\n", ret);
		return ret;
	}

	ret = zforce_resolution(ts, pdata->x_max, pdata->y_max);
	if (ret) {
		dev_err(&client->dev, "Unable to set resolution, %d\n", ret);
		goto error;
	}

	ret = zforce_scan_frequency(ts, 10, 50, 50);
	if (ret) {
		dev_err(&client->dev, "Unable to set scan frequency, %d\n", ret);
		goto error;
	}

	if (zforce_setconfig(ts, 0)) {
		dev_err(&client->dev, "Unable to set config\n");
		goto error;
	}

	/* start sending touch events */
	ret = zforce_command(ts, COMMAND_DATAREQUEST);
	if (ret) {
		dev_err(&client->dev, "Unable to request data\n");
		goto error;
	}

	/* Per NN, initial cal. take max. of 200msec.
	 * Allow time to complete this calibration
	 */
	msleep(200);

	/* FIXME: not for uptream, start hang check */
	schedule_delayed_work(&ts->check, HZ * 10);

	return 0;

error:
	zforce_command_wait(ts, COMMAND_DEACTIVATE);
	/* FIXME: not for upstream */
	ts->stopped = true;
	return ret;
}

static int zforce_stop(struct zforce_ts *ts)
{
	struct i2c_client *client = ts->client;
	int ret;

	/* FIXME: not for upstream, cancel the hang check */
	cancel_delayed_work_sync(&ts->check);

	dev_dbg(&client->dev, "stopping device\n");

	/* deactivates touch sensing and puts the device into sleep */
	ret = zforce_command_wait(ts, COMMAND_DEACTIVATE);
	if (ret != 0) {
		dev_err(&client->dev, "could not deactivate device, %d\n",
			ret);
		return ret;
	}

	/* FIXME: not for upstream */
	ts->stopped = true;

	return 0;
}

/* FIXME: not for upstream */
static void zforce_check_work(struct work_struct *work)
{
	struct zforce_ts *ts = container_of(work, struct zforce_ts, check.work);
	struct i2c_client *client = ts->client;
	int ret;

	dev_dbg(&client->dev, "periodic hang check\n");

	if (ts->stopped) {
		dev_warn(&client->dev, "zforce is stopped, doing nothing in check_work\n");
		return;
	}

	ret = zforce_command_wait(ts, COMMAND_STATUS);
	if (ret < 0) {
		dev_err(&client->dev, "could not get device status, %d\n", ret);
	}

	schedule_delayed_work(&ts->check, HZ * 10);
}

/* FIXME: not for upstream */
static void zforce_reset_work(struct work_struct *work)
{
	struct zforce_ts *ts = container_of(work, struct zforce_ts, reset.work);
	struct i2c_client *client = ts->client;
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;

	dev_dbg(&client->dev, "resetting controller\n");

	ts->stopped = true;
	cancel_delayed_work_sync(&ts->check);

	/* bring the ts into the reset state */
	printk("zforce reset\n");
	gpio_set_value(pdata->gpio_rst, 0);
	msleep(200);
	gpio_set_value(pdata->gpio_rst, 1);

	ts->command_waiting = NOTIFICATION_BOOTCOMPLETE;
	if (wait_for_completion_timeout(&ts->command_done, WAIT_TIMEOUT) == 0)
		dev_warn(&client->dev, "bootcomplete timed out\n");

	/* restart it */
	zforce_start(ts);
}

static int zforce_touch_event(struct zforce_ts *ts, u8* payload)
{
	struct i2c_client *client = ts->client;
	struct zforce_point point[ZFORCE_REPORT_POINTS];
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;
	int count, i;

	count = payload[0];
	if (count > ZFORCE_REPORT_POINTS) {
		dev_warn(&client->dev, "to many coordinates %d, expected max %d\n",
			 count, ZFORCE_REPORT_POINTS);
		count = ZFORCE_REPORT_POINTS;
	}

	for (i = 0; i < count; i++) {
		point[i].coord_x =
			payload[9 * i + 2] << 8 | payload[9 * i + 1];
		point[i].coord_y =
			payload[9 * i + 4] << 8 | payload[9 * i + 3];

		if (point[i].coord_x > pdata->x_max ||
		    point[i].coord_y > pdata->y_max) {
			dev_warn(&client->dev, "coordinates (%d,%d) invalid\n",
				point[i].coord_x, point[i].coord_y);
			point[i].coord_x = point[i].coord_y = 0;

			printk("[%s-%d] zforce got confused, scheduling reset\n", __func__, __LINE__);
			schedule_delayed_work(&ts->reset, HZ / 10);
		}

		point[i].state = payload[9 * i + 5] & 0x03;
		point[i].id = (payload[9 * i + 5] & 0xfc) >> 2;

		/* determine touch major, minor and orientation */
		point[i].area_major = max(payload[9 * i + 6], payload[9 * i + 7]);
		point[i].area_minor = min(payload[9 * i + 6], payload[9 * i + 7]);
		point[i].orientation = payload[9 * i + 6] > payload[9 * i + 7];

		point[i].pressure = payload[9 * i + 8];
		point[i].prblty = payload[9 * i + 9];
	}

	for (i = 0; i < count; i++) {
		dev_dbg(&client->dev, "point %d/%d: state %d, id %d, pressure %d, prblty %d, x %d, y %d, amajor %d, aminor %d, ori %d\n",
			i, count, point[i].state, point[i].id,
			point[i].pressure, point[i].prblty,
			point[i].coord_x, point[i].coord_y,
			point[i].area_major, point[i].area_minor,
			point[i].orientation);

		/* the zforce id starts with "1", so needs to be decreased */
		input_mt_slot(ts->input, point[i].id - 1);

		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER,
						point[i].state != STATE_UP);

		if (point[i].state != STATE_UP) {
			input_report_abs(ts->input, ABS_MT_POSITION_X,
					 point[i].coord_x);
			input_report_abs(ts->input, ABS_MT_POSITION_Y,
					 point[i].coord_y);
			input_report_abs(ts->input, ABS_MT_TOUCH_MAJOR,
					 point[i].area_major);
			input_report_abs(ts->input, ABS_MT_TOUCH_MINOR,
					 point[i].area_minor);
			input_report_abs(ts->input, ABS_MT_ORIENTATION,
					 point[i].orientation);
		}
	}

/*
	if (point[0].state != STATE_UP) {
		input_report_abs(ts->input, ABS_X, point[0].coord_x);
		input_report_abs(ts->input, ABS_Y, point[0].coord_y);
	}
 * for some unknown reasion ntx swapped the correct coordinates from the
 * zforce-ic in their driver (x <-> y),
 * only to swap them back via the tslib pointercal :-S .
 * So to stay compatible for a while do the same.
 * FIXME: resolve this
 */
	input_report_abs(ts->input, ABS_X, point[0].coord_y);
	input_report_abs(ts->input, ABS_Y, pdata->x_max - point[0].coord_x);


	input_report_key(ts->input, BTN_TOUCH, point[0].state != STATE_UP);

	/* FIXME: not for upstream, but for old tslib versions */
	input_report_abs(ts->input, ABS_PRESSURE, point[0].state != STATE_UP ? 1024 : 0);

	input_sync(ts->input);

	return 0;
}

static int zforce_read_packet(struct zforce_ts *ts, u8 *buf)
{
	struct i2c_client *client = ts->client;
	int ret;

	mutex_lock(&ts->access_mutex);

	/* read 2 byte message header */
	ret = i2c_master_recv(client, buf, 2);
	if (ret < 0) {
		dev_err(&client->dev, "error reading header: %d\n", ret);
		goto unlock;
	}

	if (buf[PAYLOAD_HEADER] != FRAME_START) {
		dev_err(&client->dev, "invalid frame start: %d\n", buf[0]);
		ret = -EIO;
		goto unlock;
	}

	if (buf[PAYLOAD_LENGTH] <= 0 || buf[PAYLOAD_LENGTH] > 255) {
		dev_err(&client->dev, "invalid payload length: %d\n",
			buf[PAYLOAD_LENGTH]);
		ret = -EIO;
		goto unlock;
	}

	/* read the message */
	ret = i2c_master_recv(client, &buf[PAYLOAD_BODY], buf[PAYLOAD_LENGTH]);
	if (ret < 0) {
		dev_err(&client->dev, "error reading payload: %d\n", ret);
		goto unlock;
	}

	dev_dbg(&client->dev, "read %d bytes for response command 0x%x\n",
		buf[PAYLOAD_LENGTH], buf[PAYLOAD_BODY]);

unlock:
	mutex_unlock(&ts->access_mutex);
	return ret;
}

static void zforce_complete(struct zforce_ts *ts, int cmd, int result)
{
	struct i2c_client *client = ts->client;

	if (ts->command_waiting == cmd) {
		dev_dbg(&client->dev, "completing command 0x%x\n", cmd);
		ts->command_result = result;
		complete(&ts->command_done);
	} else {
		dev_dbg(&client->dev, "command %d not for us\n", cmd);
	}
}

/* FIXME: not for upstream */
extern int gSleep_Mode_Suspend;

/*
 * Possible deadlock. Threads are frozen first, so if an interrupt
 * happens after this, but before the system fully sleeps, the
 * interrupt may start, making handle_level_irq mask the irq and wait
 * for the interrupt thread to start, which only happens _after_ the next
 * resume. So until this happens the irq is masked making the screen effectively
 * dead. To fix this, check the suspend state in the non-threaded irq handler
 * to send the wakeup-event and let the system resume to handle the irq then.
 */
static irqreturn_t zforce_irq(int irq, void *dev_id)
{
	struct zforce_ts *ts = dev_id;
	struct i2c_client *client = ts->client;

	if (ts->suspended) {
		/* FIXME: remove gSleep_Mode_Suspend condition */
		if (device_may_wakeup(&client->dev) || !gSleep_Mode_Suspend)
			pm_wakeup_event(&client->dev, 500);
	}

	return IRQ_WAKE_THREAD;
}

static irqreturn_t zforce_irq_thread(int irq, void *dev_id)
{
	struct zforce_ts *ts = dev_id;
	struct i2c_client *client = ts->client;
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;
	int ret;
	u8 payload_buffer[512];
	u8 *payload;

	/* when already suspended, we're holding the access_mutex, so emit
	 * a wakeup signal if necessary and return
	 */
	if (ts->suspended) {
		msleep(20);
		return IRQ_HANDLED;
	}

	dev_dbg(&client->dev, "handling interrupt\n");

	/* FIXME: remove gSleep_Mode_Suspend condition */
	/* Don't emit wakeup events from commands running during suspend */
	if (!ts->suspending && (device_may_wakeup(&client->dev) || !gSleep_Mode_Suspend))
		pm_stay_awake(&client->dev);

	while(!gpio_get_value(pdata->gpio_int)) {
		ret = zforce_read_packet(ts, payload_buffer);
		if (ret < 0) {
			dev_err(&client->dev, "could not read packet, ret: %d\n", ret); 
			break;
		}

		payload =  &payload_buffer[PAYLOAD_BODY];

		switch (payload[RESPONSE_ID]) {
		case NOTIFICATION_TOUCH:
			/* FIXME: remove gSleep_Mode_Suspend condition */
			/* Always report touch-events received when
			 * suspending, when being a wakeup source
			 */
			if (ts->suspending && (device_may_wakeup(&client->dev) || !gSleep_Mode_Suspend))
				pm_wakeup_event(&client->dev, 500);
			zforce_touch_event(ts, &payload[RESPONSE_DATA]);
			break;
		case NOTIFICATION_BOOTCOMPLETE:
			ts->boot_complete = payload[RESPONSE_DATA];
			zforce_complete(ts, payload[RESPONSE_ID], 0);
			break;
		case RESPONSE_INITIALIZE:
		case RESPONSE_DEACTIVATE:
		case RESPONSE_SETCONFIG:
		case RESPONSE_RESOLUTION:
		case RESPONSE_SCANFREQ:
			zforce_complete(ts, payload[RESPONSE_ID], payload[RESPONSE_DATA]);
			break;
		case RESPONSE_STATUS:
			/* Version Payload Results
			 * [2:major] [2:minor] [2:build] [2:rev]
			 */
			ts->version_major = (payload[RESPONSE_DATA + 1] << 8) | payload[RESPONSE_DATA];
			ts->version_minor = (payload[RESPONSE_DATA + 3] << 8) | payload[RESPONSE_DATA + 2];
			ts->version_build = (payload[RESPONSE_DATA + 5] << 8) | payload[RESPONSE_DATA + 4];
			ts->version_rev   = (payload[RESPONSE_DATA + 7] << 8) | payload[RESPONSE_DATA + 6];
			dev_info(&ts->client->dev, "Firmware Version %04x:%04x %04x:%04x\n", ts->version_major, ts->version_minor, ts->version_build, ts->version_rev);

			zforce_complete(ts, payload[RESPONSE_ID], 0);

			/* 255 is the value contained in buf9 when the zforce
			 * controller loses any valid state.
			 * FIXME: probably not for upstream
			 */
			if (payload[RESPONSE_DATA + 8] == 255) {
				printk("[%s-%d] zforce got confused, scheduling reset\n", __func__, __LINE__);
				schedule_delayed_work(&ts->reset, HZ / 10);
			}
			break;
		case NOTIFICATION_INVALID_COMMAND:
			dev_err(&ts->client->dev, "invalid command: 0x%x\n", payload[RESPONSE_DATA]);
			break;
		case NOTIFICATION_OVERRUN:
			dev_err(&ts->client->dev, "command overrun, last transaction aborted\n");
			break;
		default:
			dev_err(&ts->client->dev, "unrecognized response id: 0x%x\n", payload[RESPONSE_ID]);
			break;
		}
	}

	/* FIXME: remove gSleep_Mode_Suspend condition */
	if (!ts->suspending && (device_may_wakeup(&client->dev) || !gSleep_Mode_Suspend))
		pm_relax(&client->dev);

	dev_dbg(&client->dev, "finished interrupt\n");

	return IRQ_HANDLED;
}

static int zforce_input_open(struct input_dev *dev)
{
	struct zforce_ts *ts = input_get_drvdata(dev);
	int ret;

	ret = zforce_start(ts);
	if (ret)
		return ret;

	return 0;
}

static void zforce_input_close(struct input_dev *dev)
{
	struct zforce_ts *ts = input_get_drvdata(dev);
	struct i2c_client *client = ts->client;
	int ret;

	ret = zforce_stop(ts);
	if (ret)
		dev_warn(&client->dev, "stopping zforce failed\n");

	return;
}

#ifdef CONFIG_PM_SLEEP
static int zforce_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct zforce_ts *ts = i2c_get_clientdata(client);
	struct input_dev *input = ts->input;
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;
	int ret = 0;

	/* FIXME: should probably go away, as the irq-handler should trigger
	 * a wakup event that would resume the driver directly again
	 */
	if (!gpio_get_value(pdata->gpio_int)) {
		dev_err(&client->dev, "data request pending during suspend, this should not happen\n");
		return -EBUSY;
	}

	mutex_lock(&input->mutex);
	ts->suspending = true;

	/* FIXME: not for upstream */
	if (input->users)
		cancel_delayed_work_sync(&ts->check);

	/* when configured as wakeup source, device should always wake system
	 * therefore start device if necessary
	 */
	/* FIXME: remove gSleep_Mode_Suspend condition */
	if (device_may_wakeup(&client->dev) || !gSleep_Mode_Suspend) {
		dev_dbg(&client->dev, "suspend while being a wakeup source\n");

		/* need to start device if not open, to be wakeup source */
		if (!input->users) {
			ret = zforce_start(ts);
			if (ret)
				goto unlock;
		}

		enable_irq_wake(client->irq);
	} else if (input->users) {
		dev_dbg(&client->dev, "suspend without being a wakeup source\n");

		ret = zforce_stop(ts);
		if (ret)
			goto unlock;

		disable_irq(client->irq);
	}

	ts->suspended = true;

unlock:
	ts->suspending = false;
	mutex_unlock(&input->mutex);

	return ret;
}

static int zforce_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct zforce_ts *ts = i2c_get_clientdata(client);
	struct input_dev *input = ts->input;
	int ret = 0;

	mutex_lock(&input->mutex);

	ts->suspended = false;

	/* FIXME: remove gSleep_Mode_Suspend condition */
	if (device_may_wakeup(&client->dev) || !gSleep_Mode_Suspend) {
		dev_dbg(&client->dev, "resume from being a wakeup source\n");

		disable_irq_wake(client->irq);

		/* need to stop device if it was not open on suspend */
		if (!input->users) {
			ret = zforce_stop(ts);
			if (ret)
				goto unlock;
		}
	} else if (input->users) {
		dev_dbg(&client->dev, "resume without being a wakeup source\n");

		enable_irq(client->irq);

		ret = zforce_start(ts);
		if (ret < 0)
			goto unlock;
	}

	/* FIXME: not for upstream
	 * We schedule a nearly immediate (100ms) check to make sure,
	 * the zforce is ok after waking up.
	 */
	if (input->users)
		schedule_delayed_work(&ts->check, HZ / 10);

unlock:
	mutex_unlock(&input->mutex);

	return ret;
}

static int zforce_suspend_noirq(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;

	if (device_may_wakeup(&client->dev) || !gSleep_Mode_Suspend) {
		if(gpio_get_value(pdata->gpio_int) == 0) {
			dev_warn(&client->dev, "data waiting, aborting suspend\n");
			return -EBUSY;
		}
	}

	return 0;
}
#endif

//static SIMPLE_DEV_PM_OPS(zforce_pm_ops, zforce_suspend, zforce_resume);
static const struct dev_pm_ops zforce_pm_ops = {
	.suspend = zforce_suspend,
	.suspend_noirq = zforce_suspend_noirq,
	.resume = zforce_resume,
};

static int zforce_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	const struct zforce_ts_platdata *pdata = client->dev.platform_data;
	struct zforce_ts *ts;
	struct input_dev *input_dev;
	int ret;

	if (!pdata)
		return -EINVAL;

	ts = devm_kzalloc(&client->dev, sizeof(struct zforce_ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;

	ret = gpio_request_one(pdata->gpio_int, GPIOF_IN, "zforce_ts_int");
	if (ret) {
		dev_err(&client->dev, "request of gpio %d failed, %d\n",
			pdata->gpio_int, ret);
		return ret;
	}

	ret = gpio_request_one(pdata->gpio_rst, GPIOF_OUT_INIT_LOW, "zforce_ts_rst");
	if (ret) {
		dev_err(&client->dev, "request of gpio %d failed, %d\n",
			pdata->gpio_rst, ret);
		goto err_gpio_rst;
	}
	msleep(20);

	mutex_init(&ts->access_mutex);
	mutex_init(&ts->command_mutex);
	ts->client = client;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "could not allocate input device\n");
		ret = -ENOMEM;
		goto err_input_alloc;
	}

	ts->input = input_dev;

	input_dev->name = "Neonode zForce touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	input_dev->open = zforce_input_open;
	input_dev->close = zforce_input_close;

	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, pdata->y_max, 0, 0);

	/* FIXME: not for upstream, but for old tslib versions */
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, 1048, 0, 0);

	/* For multi touch */
	input_mt_init_slots(input_dev, ZFORCE_REPORT_POINTS);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0,
			     pdata->y_max, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0,
			     ZFORCE_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MINOR, 0,
			     ZFORCE_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_ORIENTATION, 0, 1, 0, 0);

	input_set_drvdata(ts->input, ts);

	/* FIXME: not for upstream */
	ts->stopped = true;

	init_completion(&ts->command_done);

	/* FIXME: not for upstream */
	INIT_DELAYED_WORK(&ts->check, zforce_check_work);
	INIT_DELAYED_WORK(&ts->reset, zforce_reset_work);

	/* The zforce pulls the interrupt low when it has data ready.
	 * After it is triggered the isr thread runs until all the available
	 * packets have been read and the interrupt is high again.
	 * Therefore we can trigger the interrupt anytime it is low and do
	 * not need to limit it to the interrupt edge.
	 */
	ret = request_threaded_irq(client->irq, zforce_irq, zforce_irq_thread,
				   IRQF_TRIGGER_LOW | IRQF_ONESHOT,
				   input_dev->name, ts);
	if (ret) {
		dev_err(&client->dev, "irq %d request failed\n", client->irq);
		goto err_irq_request;
	}

	i2c_set_clientdata(client, ts);

	/* let the controller boot */
	gpio_set_value(pdata->gpio_rst, 1);

	ts->command_waiting = NOTIFICATION_BOOTCOMPLETE;
	if (wait_for_completion_timeout(&ts->command_done, WAIT_TIMEOUT) == 0)
		dev_warn(&client->dev, "bootcomplete timed out\n");

	/* need to start device to get version information */
	ret = zforce_command_wait(ts, COMMAND_INITIALIZE);
	if (ret) {
		dev_err(&client->dev, "unable to initialize, %d\n", ret);
		goto err_input_register;
	}

	/* this gets the firmware version among other informations */
	ret = zforce_command_wait(ts, COMMAND_STATUS);
	if (ret < 0) {
		dev_err(&client->dev, "couldn't get status, %d\n", ret);
		zforce_stop(ts);
		goto err_input_register;
	}

	/* stop device and put it into sleep until it is opened */
	ret = zforce_stop(ts);
	if (ret < 0)
		goto err_input_register;

	device_set_wakeup_capable(&client->dev, true);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "could not register input device, %d\n",
			ret);
		goto err_input_register;
	}

	return 0;
err_input_register:
	free_irq(client->irq, ts);
err_irq_request:
	input_free_device(input_dev);
err_input_alloc:
	gpio_free(pdata->gpio_rst);
err_gpio_rst:
	gpio_free(pdata->gpio_int);

	return ret;
}

static int zforce_remove(struct i2c_client *client)
{
	struct zforce_ts *ts = i2c_get_clientdata(client);
	struct zforce_ts_platdata *pdata = client->dev.platform_data;

	input_unregister_device(ts->input);
	free_irq(client->irq, ts);
	gpio_free(pdata->gpio_rst);
	gpio_free(pdata->gpio_int);

	return 0;
}

static struct i2c_device_id zforce_idtable[] = {
	{ "zforce-ts", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, zforce_idtable);

static struct i2c_driver zforce_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "zforce-ts",
		.pm	= &zforce_pm_ops,
	},
	.probe		= zforce_probe,
	.remove		= __devexit_p(zforce_remove),
	.id_table	= zforce_idtable,
};

static int __init zforce_init(void)
{
	return i2c_add_driver(&zforce_driver);
}
module_init(zforce_init);

static void __exit zforce_exit(void)
{
	i2c_del_driver(&zforce_driver);
}
module_exit(zforce_exit);

MODULE_AUTHOR("Heiko Stuebner <heiko@sntech.de>");
MODULE_DESCRIPTION("zForce TouchScreen Driver");
MODULE_LICENSE("GPL");
