/*
* Simple driver for Texas Instruments LM3630A Backlight driver chip
* Copyright (C) 2012 Texas Instruments
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
*/
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/pwm.h>
#include <linux/platform_data/lm3630a_bl.h>
#include <linux/regulator/consumer.h>

#define REG_CTRL	0x00
#define REG_BOOST	0x02
#define REG_CONFIG	0x01
#define REG_BRT_A	0x03
#define REG_BRT_B	0x04
#define REG_I_A		0x05
#define REG_I_B		0x06
#define REG_INT_STATUS	0x09
#define REG_INT_EN	0x0A
#define REG_FAULT	0x0B
#define REG_PWM_OUTLOW	0x12
#define REG_PWM_OUTHIGH	0x13
#define REG_MAX		0x1F

#define INT_DEBOUNCE_MSEC	10
struct lm3630a_chip {
	struct device *dev;
#if 0
	struct delayed_work work;
	int irq;
	struct workqueue_struct *irqthread;
#endif
	struct i2c_client *client;
	struct lm3630a_platform_data *pdata;
	struct backlight_device *bleda;
	struct backlight_device *bledb;
	struct backlight_device *bled;
	struct pwm_device *pwmd;
	int frontlight_table;
};

static struct lm3630a_chip *gpchip;

static const unsigned char bank_a_percent[] = {
	1, 3, 7, 14, 26, 30, 41, 52, 60, 66, 				// 1.. 10
	71, 75, 80, 85, 88, 92, 96, 103, 109, 114, 			// 11 .. 20
	118, 122, 126, 129, 131, 134, 135, 138, 141, 147,	// 21 .. 30
	150, 153, 157, 158, 159, 161, 164, 166, 168, 170,	// 31 .. 40
	172, 174, 175, 177, 178, 179, 184, 185, 187, 189,	// 41 .. 50
	191, 193, 194, 196, 198, 200, 201, 203, 204, 206,	// 51 .. 60
	207, 208, 210, 211, 213, 215, 216, 217, 218, 219,	// 61 .. 70
	220, 221, 222, 223, 224, 226, 227					// 71 .. 77
};

static const unsigned char bank_b_percent[] = {
	1, 2, 6, 11, 21, 25, 35, 47, 54, 60,				// 1.. 10
	65, 69, 73, 77, 80, 86, 90, 96, 103, 108,           // 11 .. 20
	112, 116, 120, 123, 125, 127, 128, 132, 135, 140,   // 21 .. 30
	144, 147, 150, 151, 152, 154, 157, 159, 161, 163,   // 31 .. 40
	165, 167, 170, 172, 173, 174, 177, 178, 180, 182,   // 41 .. 50
	184, 186, 187, 189, 191, 193, 194, 195, 197, 198,   // 51 .. 60
	199, 201, 203, 204, 206, 208, 209, 210, 211, 212,   // 61 .. 70
	213, 214, 215, 216, 217, 219, 220,                  // 71 .. 77
};

static int gLM3630a_Fl_Table;
#define LM3630A_COLOR_TEMPERATURES		11
unsigned char lm3630a_fl_table[LM3630A_COLOR_TEMPERATURES][2][100] = {
	{
		{	// led A 100% (full_scale 4)
			43 , 66 , 79 , 90 , 96 , 107, 114, 120, 126, 132,	// 1 ~ 10
			135, 139, 143, 145, 147, 150, 152, 154, 156, 158,	// 11 ~ 20
			160, 163, 165, 168, 170, 172, 174, 176, 178, 180,	// 21 ~ 30
			181, 183, 185, 186, 188, 190, 191, 192, 193, 194,	// 31 ~ 40
			195, 196, 197, 198, 199, 200, 201, 202, 203, 204,	// 41 ~ 50
			205, 206, 207, 208, 209, 210, 211, 212, 213, 214,	// 51 ~ 60
			215, 216, 217, 218, 219, 220, 221, 222, 223, 224,	// 61 ~ 70
			225, 226, 227, 228, 229, 230, 231, 232, 233, 234,	// 71 ~ 80
			235, 236, 237, 238, 239, 240, 241, 242, 243, 244,	// 81 ~ 90
			245, 246, 247, 248, 249, 250, 251, 252, 253, 254,	// 91 ~ 100
		},
		{},
	},

	{
		{	// led A 90% (full_scale 4)
			37 , 61 , 75 , 86 , 93 , 102, 110, 117, 123, 128,	// 1 ~ 10
			133, 137, 140, 143, 144, 147, 149, 151, 153, 155,	// 11 ~ 20
			157, 159, 162, 165, 167, 170, 173, 174, 175, 176,	// 21 ~ 30
			177, 179, 180, 181, 184, 186, 187, 188, 189, 190,	// 31 ~ 40
			191, 192, 193, 194, 195, 196, 197, 198, 199, 200,	// 41 ~ 50
			201, 202, 203, 204, 205, 206, 207, 208, 209, 210,	// 51 ~ 60
			211, 212, 213, 214, 215, 216, 217, 218, 219, 220,	// 61 ~ 70
			221, 222, 223, 224, 225, 226, 227, 229, 229, 230,	// 71 ~ 80
			231, 232, 233, 234, 235, 236, 237, 238, 239, 240,	// 81 ~ 90
			241, 242, 243, 244, 245, 246, 247, 248, 249, 250,	// 91 ~ 100
		},
		{	// led B 10% (full_scale 7)
			4  , 6  , 14 , 22 , 28 , 35 , 42 , 48 , 53 , 57 ,	// 1 ~ 10
			62 , 65 , 69 , 71 , 73 , 76 , 78 , 80 , 81 , 84 ,	// 11 ~ 20
			85 , 87 , 90 , 93 , 95 , 100, 105, 97 , 107, 108,	// 21 ~ 30
			109, 110, 111, 113, 114, 116, 117, 117, 118, 119,	// 31 ~ 40
			120, 121, 122, 123, 124, 124, 125, 126, 127, 128,	// 41 ~ 50
			128, 129, 130, 131, 132, 133, 134, 135, 136, 136,	// 51 ~ 60
			141, 141, 144, 144, 144, 142, 143, 148, 148, 151,	// 61 ~ 70
			151, 151, 151, 151, 153, 152, 156, 151, 156, 158,	// 71 ~ 80
			155, 157, 160, 160, 161, 160, 162, 163, 164, 165,	// 81 ~ 90
			166, 165, 166, 169, 168, 171, 171, 171, 173, 174,	// 91 ~ 100
		},
	},

	{
		{	// led A 80% (full_scale 4)
			 32, 57 , 70 , 80 , 89 , 98 , 105, 112, 119, 124,	// 1 ~ 10
			128, 133, 136, 138, 141, 143, 145, 147, 149, 151,	// 11 ~ 20
			153, 156, 158, 161, 163, 165, 169, 171, 173, 174,	// 21 ~ 30
			174, 175, 176, 177, 179, 181, 183, 184, 185, 186,	// 31 ~ 40
			187, 188, 189, 190, 191, 192, 193, 194, 195, 196,	// 41 ~ 50
			197, 197, 198, 199, 200, 201, 202, 203, 204, 205,	// 51 ~ 60
			207, 208, 209, 210, 211, 212, 213, 214, 215, 216,	// 61 ~ 70
			217, 218, 219, 220, 221, 222, 223, 224, 225, 226,	// 71 ~ 80
			227, 228, 229, 230, 231, 232, 233, 234, 235, 236,	// 81 ~ 90
			237, 238, 239, 240, 241, 242, 243, 244, 245, 246,	// 91 ~ 100
		},
		{	// led b 20% (full_scale 7)
			 10,  25,  35,  42,  48,  56,  63,  70,  75,  79,	// 1 ~ 10
			 85,  88,  92,  94,  96,  98, 100, 102, 104, 106,	// 11 ~ 20
			108, 110, 112, 115, 117, 119, 125, 127, 134, 122,	// 21 ~ 30
			129, 132, 133, 134, 136, 139, 139, 140, 140, 142,	// 31 ~ 40
			142, 143, 145, 145, 146, 147, 148, 149, 150, 151,	// 41 ~ 50
			149, 152, 153, 153, 155, 155, 156, 157, 158, 161,	// 51 ~ 60
			160, 161, 162, 163, 164, 165, 166, 167, 168, 169,	// 61 ~ 70
			170, 171, 171, 174, 175, 176, 176, 177, 177, 178,	// 71 ~ 80
			179, 180, 181, 182, 183, 184, 185, 186, 187, 188,	// 81 ~ 90
			189, 190, 191, 192, 193, 194, 195, 196, 197, 198,	// 91 ~ 100
		},
	},

	{
		{	// led A 70% (full_scale 4)
			 23,  50,  65,  74,  84,  94, 101, 108, 114, 120,	// 1 ~ 10
			125, 128, 132, 134, 136, 139, 141, 143, 144, 147,	// 11 ~ 20
			148, 151, 153, 156, 158, 160, 165, 167, 170, 172,	// 21 ~ 30
			173, 174, 174, 175, 176, 177, 177, 178, 179, 181,	// 31 ~ 40
			182, 183, 184, 185, 186, 187, 188, 189, 190, 191,	// 41 ~ 50
			192, 193, 194, 195, 196, 197, 198, 199, 200, 201,	// 51 ~ 60
			202, 203, 204, 205, 206, 207, 208, 209, 210, 211,	// 61 ~ 70
			212, 213, 214, 215, 216, 217, 218, 219, 220, 221,	// 71 ~ 80
			222, 223, 224, 225, 226, 227, 228, 229, 230, 231,	// 81 ~ 90
			232, 233, 234, 235, 236, 237, 238, 239, 240, 241,	// 91 ~ 100
		},
		{	// led B 30% (full_scale 7)
			 22,  37,  47,  54,  60,  69,  76,  84,  87,  93,	// 1 ~ 10
			 97, 101, 104, 107, 109, 111, 113, 115, 117, 119,	// 11 ~ 20
			120, 123, 125, 128, 129, 131, 136, 137, 141, 142,	// 21 ~ 30
			147, 137, 141, 145, 149, 152, 152, 153, 154, 155,	// 31 ~ 40
			156, 157, 158, 159, 160, 161, 162, 163, 164, 165,	// 41 ~ 50
			166, 166, 166, 167, 168, 169, 170, 171, 172, 173,	// 51 ~ 60
			174, 175, 176, 177, 178, 179, 180, 181, 182, 183,	// 61 ~ 70
			184, 185, 186, 187, 188, 188, 189, 190, 191, 192,	// 71 ~ 80
			193, 194, 195, 196, 197, 197, 198, 199, 200, 201,	// 81 ~ 90
			202, 202, 203, 204, 205, 206, 207, 208, 209, 210,	// 91 ~ 100
		},
	},

	{
		{	// led A 60% (full_scale 4)
			 11,  43,  59,  69,  76,  89,  96, 103, 108, 114,	// 1 ~ 10
			119, 121, 126, 128, 131, 134, 135, 138, 140, 142,	// 11 ~ 20
			143, 146, 148, 151, 153, 155, 159, 162, 165, 167,	// 21 ~ 30
			168, 169, 171, 172, 173, 173, 173, 174, 174, 175,	// 31 ~ 40
			176, 177, 178, 179, 180, 181, 182, 183, 184, 185,	// 41 ~ 50
			186, 187, 188, 189, 190, 191, 192, 193, 194, 195,	// 51 ~ 60
			196, 197, 198, 199, 200, 201, 202, 203, 204, 205,	// 61 ~ 70
			206, 207, 208, 209, 210, 211, 212, 213, 214, 215,	// 71 ~ 80
			216, 217, 218, 219, 220, 221, 222, 223, 224, 225,	// 81 ~ 90
			226, 227, 228, 229, 230, 231, 232, 233, 234, 235,	// 91 ~ 100
		},
		{	// led B 40% (full_scale 7)
			 30,  46,  56,  63,  70,  78,  85,  92,  97, 102,	// 1 ~ 10
			107, 110, 114, 116, 118, 120, 122, 124, 126, 128,	// 11 ~ 20
			130, 132, 134, 137, 139, 141, 145, 147, 150, 152,	// 21 ~ 30
			154, 155, 156, 157, 159, 161, 164, 158, 163, 165,	// 31 ~ 40
			165, 166, 168, 168, 170, 170, 171, 172, 173, 174,	// 41 ~ 50
			175, 175, 176, 176, 178, 179, 180, 181, 182, 182,	// 51 ~ 60
			184, 185, 186, 187, 188, 188, 189, 190, 191, 192,	// 61 ~ 70
			193, 194, 195, 196, 197, 198, 199, 200, 201, 202,	// 71 ~ 80
			202, 203, 204, 205, 206, 207, 208, 209, 210, 210,	// 81 ~ 90
			211, 212, 213, 214, 215, 216, 217, 218, 219, 220,	// 91 ~ 100
		},
	},

	{
		{	// led A 50% (full_scale 4)
			  4,  33,  50,  61,  70,  80,  89,  97, 102, 108,	// 1 ~ 10
			112, 116, 120, 121, 125, 127, 129, 132, 134, 135,	// 11 ~ 20
			137, 140, 143, 145, 147, 151, 153, 155, 158, 160,	// 21 ~ 30
			162, 163, 165, 166, 167, 170, 171, 172, 173, 173,	// 31 ~ 40
			173, 173, 174, 174, 174, 176, 176, 177, 177, 178,	// 41 ~ 50
			179, 180, 181, 182, 183, 184, 185, 186, 187, 188,	// 51 ~ 60
			189, 190, 191, 192, 193, 194, 195, 196, 197, 198,	// 61 ~ 70
			199, 200, 201, 202, 203, 204, 206, 207, 208, 209,	// 71 ~ 80
			210, 211, 212, 213, 214, 215, 216, 217, 218, 219,	// 81 ~ 90
			220, 221, 222, 223, 224, 225, 226, 227, 228, 229,	// 91 ~ 100
		},
		{	// led B 50% (full_scale 7)
			 38, 52 , 62 , 70 , 77 , 86 , 93 , 99 , 104, 110,	// 1 ~ 10
			114, 117, 120, 123, 125, 127, 129, 132, 133, 135,	// 11 ~ 20
			137, 140, 142, 145, 146, 147, 152, 155, 158, 160,	// 21 ~ 30
			161, 162, 164, 165, 167, 169, 169, 170, 171, 173,	// 31 ~ 40
			174, 176, 173, 174, 177, 177, 179, 180, 181, 182,	// 41 ~ 50
			182, 184, 184, 185, 186, 187, 188, 189, 190, 190,	// 51 ~ 60
			191, 192, 193, 194, 195, 196, 197, 198, 199, 200,	// 61 ~ 70
			201, 202, 202, 203, 204, 205, 206, 207, 208, 209,	// 71 ~ 80
			210, 211, 211, 212, 213, 214, 215, 216, 217, 218,	// 81 ~ 90
			219, 221, 221, 222, 223, 224, 225, 226, 227, 228,	// 91 ~ 100
		},
	},

	{
		{	// led A 40% (full_scale 4)
			  4, 18 , 40 , 51 , 60 , 70 , 82 , 89 , 95 , 100,	// 1 ~ 10
			105, 108, 112, 114, 117, 120, 122, 124, 126, 128,	// 11 ~ 20
			130, 133, 134, 137, 140, 142, 146, 148, 151, 153,	// 21 ~ 30
			155, 156, 157, 159, 160, 162, 162, 164, 165, 165,	// 31 ~ 40
			167, 168, 169, 170, 171, 172, 173, 173, 173, 173,	// 41 ~ 50
			174, 174, 174, 174, 174, 175, 176, 177, 178, 179,	// 51 ~ 60
			180, 181, 182, 184, 184, 186, 187, 188, 189, 189,	// 61 ~ 70
			191, 191, 192, 193, 194, 195, 196, 197, 198, 199,	// 71 ~ 80
			200, 201, 202, 203, 204, 206, 207, 208, 209, 210,	// 81 ~ 90
			211, 212, 213, 214, 215, 216, 217, 218, 219, 220,	// 91 ~ 100
		},
		{	// led B 60% (full_scale 7)
			 42,  58,  68,  77,  84,  92,  98, 105, 110, 115,	// 1 ~ 10
			120, 123, 126, 129, 130, 133, 135, 137, 139, 141,	// 11 ~ 20
			143, 145, 148, 150, 152, 154, 158, 161, 164, 166,	// 21 ~ 30
			167, 168, 170, 171, 173, 175, 176, 176, 177, 179,	// 31 ~ 40
			179, 180, 181, 182, 184, 185, 186, 187, 188, 189,	// 41 ~ 50
			187, 188, 189, 190, 191, 192, 193, 194, 195, 196,	// 51 ~ 60
			197, 198, 199, 200, 201, 202, 203, 204, 205, 206,	// 61 ~ 70
			207, 208, 209, 210, 211, 212, 213, 214, 215, 216,	// 71 ~ 80
			217, 218, 219, 220, 221, 222, 223, 224, 225, 226,	// 81 ~ 90
			227, 228, 229, 230, 231, 232, 233, 234, 235, 236,	// 91 ~ 100
		},
	},

	{
		{	// led A 30% (full_scale 4)
			  4,   4,  24,  37,  48,  60,  68,  70,  84,  89,	// 1 ~ 10
			 95,  99, 103, 105, 107, 110, 112, 114, 116, 118,	// 11 ~ 20
			120, 123, 125, 127, 129, 132, 135, 138, 142, 143,	// 21 ~ 30
			144, 146, 145, 149, 150, 151, 152, 152, 154, 155,	// 31 ~ 40
			156, 157, 160, 161, 161, 163, 164, 165, 166, 167,	// 41 ~ 50
			166, 167, 168, 171, 171, 171, 172, 172, 173, 173,	// 51 ~ 60
			173, 173, 174, 174, 174, 175, 175, 177, 178, 179,	// 61 ~ 70
			180, 181, 181, 183, 184, 185, 186, 188, 188, 188,	// 71 ~ 80
			189, 190, 191, 192, 193, 194, 195, 196, 197, 198,	// 81 ~ 90
			199, 200, 201, 202, 203, 204, 205, 206, 207, 208,	// 91 ~ 100
		},
		{	// led B 70% (full_scale 7)
			 38,  63,  74,  81,  88,  97, 103, 110, 115, 120,	// 1 ~ 10
			124, 128, 131, 134, 136, 139, 140, 142, 144, 146,	// 11 ~ 20
			148, 151, 153, 156, 158, 160, 164, 166, 169, 171,	// 21 ~ 30
			173, 174, 176, 176, 177, 178, 179, 181, 183, 184,	// 31 ~ 40
			185, 186, 187, 188, 189, 189, 190, 191, 192, 193,	// 41 ~ 50
			194, 195, 196, 196, 197, 198, 199, 200, 201, 202,	// 51 ~ 60
			204, 206, 205, 206, 207, 208, 209, 210, 211, 212,	// 61 ~ 70
			213, 214, 215, 216, 217, 218, 219, 220, 221, 222,	// 71 ~ 80
			223, 224, 225, 226, 227, 228, 229, 230, 231, 232,	// 81 ~ 90
			233, 234, 235, 236, 237, 238, 239, 240, 241, 242,	// 91 ~ 100
		},
	},

	{
		{	// led A 20% (full_scale 4)
			  4,   4,   4,  12,  26,  40,  50,  60,  66,  73,	// 1 ~ 10
			 77,  85,  88,  90,  93,  95,  98, 100, 102, 104,	// 11 ~ 20
			106, 108, 110, 113, 115, 118, 122, 125, 127, 130,	// 21 ~ 30
			131, 132, 134, 135, 136, 139, 140, 141, 142, 143,	// 31 ~ 40
			144, 144, 145, 146, 149, 148, 151, 152, 150, 152,	// 41 ~ 50
			151, 154, 155, 155, 156, 156, 157, 158, 159, 158,	// 51 ~ 60
			159, 160, 161, 162, 163, 164, 165, 164, 165, 166,	// 61 ~ 70
			167, 168, 169, 170, 169, 170, 171, 172, 173, 174,	// 71 ~ 80
			175, 176, 177, 178, 179, 180, 181, 182, 184, 185,	// 81 ~ 90
			186, 187, 188, 189, 189, 190, 192, 193, 193, 194,	// 91 ~ 100
		},
		{	// led B 80% (full_scale 7)
			 41,  63,  78,  86,  92, 100, 108, 114, 119, 124,	// 1 ~ 10
			129, 132, 136, 138, 140, 143, 145, 147, 149, 151,	// 11 ~ 20
			153, 155, 157, 160, 162, 164, 168, 170, 174, 176,	// 21 ~ 30
			177, 178, 180, 181, 183, 185, 186, 187, 188, 188,	// 31 ~ 40
			189, 190, 191, 192, 193, 194, 195, 196, 197, 198,	// 41 ~ 50
			199, 199, 200, 201, 202, 203, 204, 205, 206, 207,	// 51 ~ 60
			208, 209, 210, 211, 212, 213, 214, 215, 216, 217,	// 61 ~ 70
			218, 219, 220, 221, 222, 223, 224, 225, 226, 226,	// 71 ~ 80
			227, 228, 229, 230, 231, 232, 233, 234, 235, 236,	// 81 ~ 90
			237, 238, 239, 240, 241, 242, 243, 244, 245, 246,	// 91 ~ 100
		},
	},

	{
		{	// led A 10% (full_scale 4)
			  4,   4,   4,   4,   4,   4,  11,  24,  34,  43,	// 1 ~ 10
			 49,  55,  60,  62,  65,  68,  70,  73,  75,  77,	// 11 ~ 20
			 79,  84,  87,  90,  92,  94, 101, 102, 105, 105,	// 21 ~ 30
			107, 108, 109, 110, 112, 114, 115, 114, 117, 119,	// 31 ~ 40
			125, 120, 122, 122, 124, 124, 125, 126, 127, 130,	// 41 ~ 50
			135, 130, 131, 131, 132, 133, 134, 138, 139, 138,	// 51 ~ 60
			137, 138, 139, 140, 141, 142, 143, 144, 145, 146,	// 61 ~ 70
			147, 148, 148, 149, 150, 151, 152, 153, 154, 155,	// 71 ~ 80
			156, 157, 158, 159, 160, 161, 162, 163, 164, 165,	// 81 ~ 90
			166, 166, 167, 168, 169, 170, 171, 172, 173, 173,	// 91 ~ 100
		},
		{	// led B 90% (full_scale 7)
			 41,  63,  77,  87,  93, 104, 111, 118, 123, 128,	// 1 ~ 10
			132, 136, 139, 142, 144, 147, 149, 151, 152, 154,	// 11 ~ 20
			157, 159, 161, 163, 166, 168, 172, 175, 177, 180,	// 21 ~ 30
			181, 182, 185, 186, 188, 189, 190, 191, 192, 193,	// 31 ~ 40
			193, 194, 195, 196, 197, 198, 199, 200, 201, 202,	// 41 ~ 50
			202, 203, 204, 205, 206, 207, 208, 208, 209, 210,	// 51 ~ 60
			211, 212, 213, 214, 215, 216, 217, 218, 219, 220,	// 61 ~ 70
			221, 222, 223, 224, 225, 226, 227, 228, 229, 230,	// 71 ~ 80
			231, 232, 233, 234, 235, 236, 237, 238, 239, 240,	// 81 ~ 90
			241, 242, 243, 244, 245, 246, 247, 248, 249, 250,	// 91 ~ 100
		},
	},

	{
		{},
		{	// led B 100% (full_scale 7)
			58 , 74 , 85 , 93 , 99 , 108, 115, 121, 126, 131,	// 1 ~ 10
			136, 139, 143, 145, 147, 150, 152, 154, 156, 158,	// 11 ~ 20
			160, 163, 165, 168, 170, 172, 174, 176, 178, 180,	// 21 ~ 30
			181, 183, 185, 186, 188, 190, 191, 192, 193, 194,	// 31 ~ 40
			195, 196, 197, 198, 199, 200, 201, 202, 203, 204,	// 41 ~ 50
			205, 206, 207, 208, 209, 210, 211, 212, 213, 214,	// 51 ~ 60
			215, 216, 217, 218, 219, 220, 221, 222, 223, 224,	// 61 ~ 70
			225, 226, 227, 228, 229, 230, 231, 232, 233, 234,	// 71 ~ 80
			235, 236, 237, 238, 239, 240, 241, 242, 243, 244,	// 81 ~ 90
			245, 246, 247, 248, 249, 250, 251, 252, 253, 254,	// 91 ~ 100
		},
	},
};

/* i2c access */
static int lm3630a_read(struct lm3630a_chip *pchip, unsigned int reg)
{
	int rval;

	rval = i2c_smbus_read_byte_data(pchip->client, reg);
	if (rval < 0)
		return rval;
	return rval & 0xFF;
}

static int lm3630a_write(struct lm3630a_chip *pchip,
			 unsigned int reg, unsigned int data)
{
//	printk("[%s-%d] reg 0x%02X, val %02X\n", __func__, __LINE__, reg, data);
	return i2c_smbus_write_byte_data(pchip->client, reg, (uint8_t)data);
}

static int lm3630a_update(struct lm3630a_chip *pchip,
			  unsigned int reg, unsigned int mask,
			  unsigned int data)
{
	int rval = i2c_smbus_read_byte_data(pchip->client, reg);
	if (rval < 0)
		return rval;
//	printk("[%s-%d] reg 0x%02X, val %02X\n", __func__, __LINE__, reg, data);
	rval &= ~mask;
	rval |= (data & mask);
	return i2c_smbus_write_byte_data(pchip->client, reg, (uint8_t)rval);
}

/* initialize chip */
static int lm3630a_chip_init(struct lm3630a_chip *pchip)
{
	int rval;
	struct lm3630a_platform_data *pdata = pchip->pdata;
	struct regulator *fl_regulator;

	fl_regulator = regulator_get(&pchip->client->dev, "vdd_fl_lm3630a");
	if (IS_ERR(fl_regulator)) {
		printk("%s, regulator \"vdd_fl_lm3630a\" not registered.(%d)\n", __func__, fl_regulator);
		return -1;
	}
	else
			printk("%s, vdd_fl_lm3630a found on channel 0\n", __func__);
	regulator_enable (fl_regulator);
	msleep (200);

	usleep_range(1000, 2000);
	/* set Filter Strength Register */
	rval = lm3630a_write(pchip, 0x50, 0x03);
	/* set Cofig. register */
	rval |= lm3630a_update(pchip, REG_CONFIG, 0x07, pdata->pwm_ctrl);
	/* set boost control */
	rval |= lm3630a_write(pchip, REG_BOOST, 0x38);
	/* set current A */
	rval |= lm3630a_update(pchip, REG_I_A, 0x1F, 0x0);
	/* set current B */
	rval |= lm3630a_write(pchip, REG_I_B, 0x0);
	/* set control */
	rval |= lm3630a_update(pchip, REG_CTRL, 0x14, pdata->leda_ctrl);
	rval |= lm3630a_update(pchip, REG_CTRL, 0x0B, pdata->ledb_ctrl);
	usleep_range(1000, 2000);
	/* set brightness A and B */
	rval |= lm3630a_write(pchip, REG_BRT_A, pdata->leda_init_brt);
	rval |= lm3630a_write(pchip, REG_BRT_B, pdata->ledb_init_brt);

	if (rval < 0)
		dev_err(pchip->dev, "i2c failed to access register\n");
	return rval;
}

#if 0
/* interrupt handling */
static void lm3630a_delayed_func(struct work_struct *work)
{
	int rval;
	struct lm3630a_chip *pchip;

	pchip = container_of(work, struct lm3630a_chip, work.work);

	rval = lm3630a_read(pchip, REG_INT_STATUS);
	if (rval < 0) {
		dev_err(pchip->dev,
			"i2c failed to access REG_INT_STATUS Register\n");
		return;
	}

	dev_info(pchip->dev, "REG_INT_STATUS Register is 0x%x\n", rval);
}

static irqreturn_t lm3630a_isr_func(int irq, void *chip)
{
	int rval;
	struct lm3630a_chip *pchip = chip;
	unsigned long delay = msecs_to_jiffies(INT_DEBOUNCE_MSEC);

	queue_delayed_work(pchip->irqthread, &pchip->work, delay);

	rval = lm3630a_update(pchip, REG_CTRL, 0x80, 0x00);
	if (rval < 0) {
		dev_err(pchip->dev, "i2c failed to access register\n");
		return IRQ_NONE;
	}
	return IRQ_HANDLED;
}

static int lm3630a_intr_config(struct lm3630a_chip *pchip)
{
	int rval;

	rval = lm3630a_write(pchip, REG_INT_EN, 0x87);
	if (rval < 0)
		return rval;

	INIT_DELAYED_WORK(&pchip->work, lm3630a_delayed_func);
	pchip->irqthread = create_singlethread_workqueue("lm3630a-irqthd");
	if (!pchip->irqthread) {
		dev_err(pchip->dev, "create irq thread fail\n");
		return -ENOMEM;
	}
	if (request_threaded_irq
	    (pchip->irq, NULL, lm3630a_isr_func,
	     IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "lm3630a_irq", pchip)) {
		dev_err(pchip->dev, "request threaded irq fail\n");
		destroy_workqueue(pchip->irqthread);
		return -ENOMEM;
	}
	return rval;
}
#endif

static void lm3630a_pwm_ctrl(struct lm3630a_chip *pchip, int br, int br_max)
{
#if 0
	unsigned int period = pwm_get_period(pchip->pwmd);
	unsigned int duty = br * period / br_max;

	pwm_config(pchip->pwmd, duty, period);
	if (duty)
		pwm_enable(pchip->pwmd);
	else
		pwm_disable(pchip->pwmd);
#endif
}

/* update and get brightness */
static int lm3630a_bank_a_update_status(struct backlight_device *bl)
{
	int ret;
	struct lm3630a_chip *pchip = bl_get_data(bl);
#if 0
	enum lm3630a_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	/* pwm control */
	if ((pwm_ctrl & LM3630A_PWM_BANK_A) != 0) {
		lm3630a_pwm_ctrl(pchip, bl->props.brightness,
				 bl->props.max_brightness);
		return bl->props.brightness;
	}
#endif

	/* disable sleep */
	ret = lm3630a_update(pchip, REG_CTRL, 0x80, 0x00);
	if (ret < 0)
		goto out_i2c_err;
	usleep_range(1000, 2000);
//	printk ("[%s-%d] brightness %d, power %d\n", __func__,__LINE__,bl->props.brightness,bl->props.power);

	/* minimum brightness is 0x04 */
	ret = lm3630a_write(pchip, REG_BRT_A, bl->props.brightness);
	if (0x20 > bl->props.power)
		ret |= lm3630a_update(pchip, REG_I_A, 0x1F, bl->props.power);

	if (bl->props.brightness < 0x1)
		ret |= lm3630a_update(pchip, REG_CTRL, LM3630A_LEDA_ENABLE, 0);
	else
		ret |= lm3630a_update(pchip, REG_CTRL,
				      LM3630A_LEDA_ENABLE, LM3630A_LEDA_ENABLE);
	if (ret < 0)
		goto out_i2c_err;
	return bl->props.brightness;

out_i2c_err:
	dev_err(pchip->dev, "i2c failed to access\n");
	return bl->props.brightness;
}

static int lm3630a_bank_a_get_brightness(struct backlight_device *bl)
{
	int brightness, rval;
	struct lm3630a_chip *pchip = bl_get_data(bl);
#if 0
	enum lm3630a_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if ((pwm_ctrl & LM3630A_PWM_BANK_A) != 0) {
		rval = lm3630a_read(pchip, REG_PWM_OUTHIGH);
		if (rval < 0)
			goto out_i2c_err;
		brightness = (rval & 0x01) << 8;
		rval = lm3630a_read(pchip, REG_PWM_OUTLOW);
		if (rval < 0)
			goto out_i2c_err;
		brightness |= rval;
		goto out;
	}
#endif
	/* disable sleep */
	rval = lm3630a_update(pchip, REG_CTRL, 0x80, 0x00);
	if (rval < 0)
		goto out_i2c_err;
	usleep_range(1000, 2000);
	rval = lm3630a_read(pchip, REG_BRT_A);
	if (rval < 0)
		goto out_i2c_err;
	brightness = rval;

out:
	bl->props.brightness = brightness;
	return bl->props.brightness;
out_i2c_err:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return 0;
}

static const struct backlight_ops lm3630a_bank_a_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3630a_bank_a_update_status,
	.get_brightness = lm3630a_bank_a_get_brightness,
};

/* update and get brightness */
static int lm3630a_bank_b_update_status(struct backlight_device *bl)
{
	int ret;
	struct lm3630a_chip *pchip = bl_get_data(bl);
#if 0
	enum lm3630a_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	/* pwm control */
	if ((pwm_ctrl & LM3630A_PWM_BANK_B) != 0) {
		lm3630a_pwm_ctrl(pchip, bl->props.brightness,
				 bl->props.max_brightness);
		return bl->props.brightness;
	}
#endif
	/* disable sleep */
	ret = lm3630a_update(pchip, REG_CTRL, 0x80, 0x00);
	if (ret < 0)
		goto out_i2c_err;
	usleep_range(1000, 2000);
//	printk ("[%s-%d] brightness %d, power %d\n", __func__,__LINE__,bl->props.brightness,bl->props.power);
	/* minimum brightness is 0x04 */
	ret = lm3630a_write(pchip, REG_BRT_B, bl->props.brightness);

	if (0x20 > bl->props.power)
		ret |= lm3630a_write(pchip, REG_I_B, bl->props.power);

	if (bl->props.brightness < 0x1)
		ret |= lm3630a_update(pchip, REG_CTRL, LM3630A_LEDB_ENABLE, 0);
	else
		ret |= lm3630a_update(pchip, REG_CTRL,
				      LM3630A_LEDB_ENABLE, LM3630A_LEDB_ENABLE);
	if (ret < 0)
		goto out_i2c_err;
	return bl->props.brightness;

out_i2c_err:
	dev_err(pchip->dev, "i2c failed to access REG_CTRL\n");
	return bl->props.brightness;
}

static int lm3630a_bank_b_get_brightness(struct backlight_device *bl)
{
	int brightness, rval;
	struct lm3630a_chip *pchip = bl_get_data(bl);
#if 0
	enum lm3630a_pwm_ctrl pwm_ctrl = pchip->pdata->pwm_ctrl;

	if ((pwm_ctrl & LM3630A_PWM_BANK_B) != 0) {
		rval = lm3630a_read(pchip, REG_PWM_OUTHIGH);
		if (rval < 0)
			goto out_i2c_err;
		brightness = (rval & 0x01) << 8;
		rval = lm3630a_read(pchip, REG_PWM_OUTLOW);
		if (rval < 0)
			goto out_i2c_err;
		brightness |= rval;
		goto out;
	}
#endif
	/* disable sleep */
	rval = lm3630a_update(pchip, REG_CTRL, 0x80, 0x00);
	if (rval < 0)
		goto out_i2c_err;
	usleep_range(1000, 2000);
	rval = lm3630a_read(pchip, REG_BRT_B);
	if (rval < 0)
		goto out_i2c_err;
	brightness = rval;

out:
	bl->props.brightness = brightness;
	return bl->props.brightness;
out_i2c_err:
	dev_err(pchip->dev, "i2c failed to access register\n");
	return 0;
}

static const struct backlight_ops lm3630a_bank_b_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3630a_bank_b_update_status,
	.get_brightness = lm3630a_bank_b_get_brightness,
};

static ssize_t led_a_per_info(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int val = gpchip->bleda->props.brightness;
	if (val) {
		for (val=0;val < sizeof(bank_a_percent);val++) {
			if (gpchip->bleda->props.brightness == bank_a_percent[val])
				break;
		}
		val++;
	}
	sprintf (buf, "%d", val);
	return strlen(buf);
}

static ssize_t led_a_per_ctrl(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int val = simple_strtoul (buf, NULL, 10);

	if (val == 0)
		gpchip->bleda->props.brightness = 0;
	else {
		if (val > sizeof (bank_a_percent))
			val = sizeof(bank_a_percent);
		gpchip->bleda->props.brightness = bank_a_percent[val-1];
	}
//	printk ("[%s-%d] set %d\%, brightness %d\n", __func__, __LINE__, val, gpchip->bleda->props.brightness);
	lm3630a_bank_a_update_status (gpchip->bleda);
	return count;
}

static DEVICE_ATTR (percent, 0644, led_a_per_info, led_a_per_ctrl);

static ssize_t led_b_per_info(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	int val = gpchip->bledb->props.brightness;
	if (val) {
		for (val=0;val < sizeof(bank_b_percent);val++) {
			if (gpchip->bledb->props.brightness == bank_b_percent[val])
				break;
		}
		val++;
	}
	sprintf (buf, "%d", val);
	return strlen(buf);
}

static ssize_t led_b_per_ctrl(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int val = simple_strtoul (buf, NULL, 10);

	if (val == 0)
		gpchip->bledb->props.brightness = 0;
	else {
		if (val > sizeof (bank_a_percent))
			val = sizeof(bank_a_percent);
		gpchip->bledb->props.brightness = bank_b_percent[val-1];
	}
//	printk ("[%s-%d] set %d\%, brightness %d\n", __func__, __LINE__, val, gpchip->bledb->props.brightness);
	lm3630a_bank_b_update_status (gpchip->bledb);
	return count;
}

static struct device_attribute dev_attr_b_percent = __ATTR(percent, 0644,led_b_per_info, led_b_per_ctrl);

static void lm3630a_set_FL (unsigned char led_A_current, unsigned char led_A_brightness,
		unsigned char led_B_current, unsigned char led_B_brightness)
{
	int ret;
	printk ("[%s-%d] led A %d, led B %d\n", __func__,__LINE__, led_A_brightness, led_B_brightness);
	/* disable sleep */
	ret = lm3630a_update(gpchip, REG_CTRL, 0x80, 0x00);
	usleep_range(1000, 2000);

	/* minimum brightness is 0x04 */
	ret = lm3630a_write(gpchip, REG_BRT_A, led_A_brightness);
	if (0x20 > led_A_current)
		ret |= lm3630a_update(gpchip, REG_I_A, 0x1F, led_A_current);

	/* minimum brightness is 0x04 */
	ret = lm3630a_write(gpchip, REG_BRT_B, led_B_brightness);
	if (0x20 > led_B_current)
		ret |= lm3630a_write(gpchip, REG_I_B, led_B_current);

	lm3630a_update(gpchip, REG_CTRL,
			LM3630A_LEDA_ENABLE|LM3630A_LEDB_ENABLE, LM3630A_LEDA_ENABLE|LM3630A_LEDB_ENABLE);
}

int fl_lm3630a_percentage (int iFL_Percentage)
{
	int iFL_table = gpchip->frontlight_table;

	if (LM3630A_COLOR_TEMPERATURES <= iFL_table) {
		printk ("[%s-%d] Front light table %d out of range.\n", __func__,__LINE__, iFL_table);
		return -1;
	}
	if (0 == iFL_Percentage)
		lm3630a_set_FL (0, 0, 0, 0);
	else
		lm3630a_set_FL (4, lm3630a_fl_table[iFL_table][0][iFL_Percentage-1],
			7, lm3630a_fl_table[iFL_table][1][iFL_Percentage-1]);
	gpchip->bled->props.brightness = iFL_Percentage;
	return 0;
}

int fl_lm3630a_set_color (int iFL_color)
{
	if (LM3630A_COLOR_TEMPERATURES <= iFL_color) {
		printk ("[%s-%d] Front light table %d out of range.\n", __func__,__LINE__, iFL_color);
		return -1;
	}
	gpchip->frontlight_table = iFL_color;
	fl_lm3630a_percentage (gpchip->bled->props.brightness);
}

/* update and get brightness */
static int lm3630a_update_status(struct backlight_device *bl)
{
	fl_lm3630a_percentage (bl->props.brightness);
	return bl->props.brightness;
}

static int lm3630a_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops lm3630a_ops = {
	.options = BL_CORE_SUSPENDRESUME,
	.update_status = lm3630a_update_status,
	.get_brightness = lm3630a_get_brightness,
};

static ssize_t led_color_get(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	sprintf (buf, "%d", gpchip->frontlight_table);
	return strlen(buf);
}

static ssize_t led_color_set(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	int val = simple_strtoul (buf, NULL, 10);

	gpchip->frontlight_table = val;
	fl_lm3630a_percentage (gpchip->bled->props.brightness);

	return count;
}

static DEVICE_ATTR (color, 0644, led_color_get, led_color_set);

static int lm3630a_backlight_register(struct lm3630a_chip *pchip)
{
	struct backlight_properties props;
	struct lm3630a_platform_data *pdata = pchip->pdata;
	int rval;

	props.type = BACKLIGHT_RAW;
	if (pdata->leda_ctrl != LM3630A_LEDA_DISABLE) {
		props.brightness = pdata->leda_init_brt;
		props.max_brightness = pdata->leda_max_brt;
		props.power = pdata->leda_full_scale;
		pchip->bleda =
			backlight_device_register("lm3630a_leda", pchip->dev, pchip,
							       &lm3630a_bank_a_ops, &props);
		if (IS_ERR(pchip->bleda))
			return PTR_ERR(pchip->bleda);

		rval = device_create_file(&pchip->bleda->dev, &dev_attr_percent);
		if (rval < 0) {
			dev_err(&pchip->bleda->dev, "fail : backlight percent create.\n");
			return rval;
		}
	}

	if ((pdata->ledb_ctrl != LM3630A_LEDB_DISABLE) &&
	    (pdata->ledb_ctrl != LM3630A_LEDB_ON_A)) {
		props.brightness = pdata->ledb_init_brt;
		props.max_brightness = pdata->ledb_max_brt;
		props.power = pdata->ledb_full_scale;
		pchip->bledb =
			backlight_device_register("lm3630a_ledb", pchip->dev, pchip,
							       &lm3630a_bank_b_ops, &props);
		if (IS_ERR(pchip->bledb))
			return PTR_ERR(pchip->bledb);

		rval = device_create_file(&pchip->bledb->dev, &dev_attr_b_percent);
		if (rval < 0) {
			dev_err(&pchip->bledb->dev, "fail : backlight percent create.\n");
			return rval;
		}
	}
	props.brightness = 100;
	props.max_brightness = 100;
	pchip->bled =
			backlight_device_register("lm3630a_led", pchip->dev, pchip,
							       &lm3630a_ops, &props);
	if (IS_ERR(pchip->bled))
		return PTR_ERR(pchip->bled);

	rval = device_create_file(&pchip->bled->dev, &dev_attr_color);
	if (rval < 0) {
		dev_err(&pchip->bled->dev, "fail : backlight color create.\n");
		return rval;
	}

	return 0;
}

static int lm3630a_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct lm3630a_platform_data *pdata = dev_get_platdata(&client->dev);
	struct lm3630a_chip *pchip;
	int rval;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "fail : i2c functionality check\n");
		return -EOPNOTSUPP;
	}

	gpchip = pchip = devm_kzalloc(&client->dev, sizeof(struct lm3630a_chip),
			     GFP_KERNEL);
	if (!pchip)
		return -ENOMEM;
	pchip->dev = &client->dev;

	pchip->client = client;

	i2c_set_clientdata(client, pchip);
	if (pdata == NULL) {
		pdata = devm_kzalloc(pchip->dev,
				     sizeof(struct lm3630a_platform_data),
				     GFP_KERNEL);
		if (pdata == NULL)
			return -ENOMEM;
		/* default values */
		pdata->leda_ctrl = LM3630A_LEDA_ENABLE;
		pdata->ledb_ctrl = LM3630A_LEDB_ENABLE;
		pdata->leda_max_brt = LM3630A_MAX_BRIGHTNESS;
		pdata->ledb_max_brt = LM3630A_MAX_BRIGHTNESS;
		pdata->leda_init_brt = 0;
		pdata->ledb_init_brt = 0;
		pdata->leda_full_scale = LM3630A_DEF_FULLSCALE;
		pdata->ledb_full_scale = LM3630A_DEF_FULLSCALE;
		pdata->pwm_ctrl = LM3630A_PWM_DISABLE;
	}
	pchip->pdata = pdata;

	/* chip initialize */
	rval = lm3630a_chip_init(pchip);
	if (rval < 0) {
		dev_err(&client->dev, "fail : init chip\n");
		return rval;
	}
	/* backlight register */
	rval = lm3630a_backlight_register(pchip);
	if (rval < 0) {
		dev_err(&client->dev, "fail : backlight register.\n");
		return rval;
	}

#if 0
	/* pwm */
	if (pdata->pwm_ctrl != LM3630A_PWM_DISABLE) {
		pchip->pwmd = devm_pwm_get(pchip->dev, "lm3630a-pwm");
		if (IS_ERR(pchip->pwmd)) {
			dev_err(&client->dev, "fail : get pwm device\n");
			return PTR_ERR(pchip->pwmd);
		}
	}

	pchip->pwmd->period = pdata->pwm_period;

	/* interrupt enable  : irq 0 is not allowed */
	pchip->irq = client->irq;
	if (pchip->irq) {
		rval = lm3630a_intr_config(pchip);
		if (rval < 0)
			return rval;
	}
#endif
	dev_info(&client->dev, "LM3630A backlight register OK.\n");
	return 0;
}

static int lm3630a_remove(struct i2c_client *client)
{
	int rval;
	struct lm3630a_chip *pchip = i2c_get_clientdata(client);

	rval = lm3630a_write(pchip, REG_BRT_A, 0);
	if (rval < 0)
		dev_err(pchip->dev, "i2c failed to access register\n");

	rval = lm3630a_write(pchip, REG_BRT_B, 0);
	if (rval < 0)
		dev_err(pchip->dev, "i2c failed to access register\n");

#if 0
	if (pchip->irq) {
		free_irq(pchip->irq, pchip);
		flush_workqueue(pchip->irqthread);
		destroy_workqueue(pchip->irqthread);
	}
#endif
	return 0;
}

static const struct i2c_device_id lm3630a_id[] = {
	{LM3630A_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, lm3630a_id);

static struct i2c_driver lm3630a_i2c_driver = {
	.driver = {
		   .name = LM3630A_NAME,
		   },
	.probe = lm3630a_probe,
	.remove = lm3630a_remove,
	.id_table = lm3630a_id,
};

static int __init lm3630a_init(void)
{
	return i2c_add_driver(&lm3630a_i2c_driver);
}

static void __exit lm3630a_exit(void)
{
	i2c_del_driver(&lm3630a_i2c_driver);
}

module_init(lm3630a_init);
module_exit(lm3630a_exit);

MODULE_DESCRIPTION("Texas Instruments Backlight driver for LM3630A");
MODULE_AUTHOR("Daniel Jeong <gshark.jeong@gmail.com>");
MODULE_AUTHOR("LDD MLP <ldd-mlp@list.ti.com>");
MODULE_LICENSE("GPL v2");
