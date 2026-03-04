/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * d6t-core.h - interfaces internal to the omron d6t
*/
#ifndef _D6T_CORE_H
#define _D6T_CORE_H

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/string.h>

//SUPPORT FLAGS
#define NOT_SUPPORT -1

//DEFINE D6T-01A MODEL
#define D6T_01A_COMMAND 0x4C

//DEFINE D6T-32L-01A MODEL
#define D6T_32L_01A_STATUS_REG 0x01
#define D6T_32L_01A_IIR_AVG_REG 0x01
#define D6T_32L_01A_CYCLE 0x02

#define D6T_32L_01A_COMMAND 0x4D

/*
@brief D6T device structure
@details This structure holds the configuration and state of a D6T device.
*/
struct d6t_t {
	const char *name; // Model name of the D6T device
	u8 row; // Number of rows in the D6T array
	u8 col; // Number of columns in the D6T array
	s8 iir; // IIR filter setting
	s8 avg; // Average filter setting
	s8 cycle; //Sampling cycle setting

	u8 command; // Command to send to the D6T device

	u8 *buffer;
	u16 n_read;

	s16 *raw_data;
	u16 n_raw_data;
};

/*
@brief Init D6T device
@param name your d6t model name
@return d6t your d6t
*/
static inline struct d6t_t *d6t_init(const char *name);

/*
@brief Exit your D6T device, support function for d6t_remove in driver
@param d6t your d6t
*/
static inline u32 d6t_clear(struct d6t_t **d6t);

/*
@brief Read data from D6T device
@param d6t your d6t
@return 0 on success, negative error code on failure
*/
static inline u32 d6t_read_data(struct i2c_client *client, struct d6t_t *d6t);

/*
@brief Read data from D6T device
@param d6t your d6t
@param reg register to read
@return value read from the register, or negative error code on failure
*/

/*
@brief Write data to D6T device
@param d6t your d6t
@param reg register to write
@param value value to write
@return 0 on success, negative error code on failure
*/
static inline u32 d6t_write(struct i2c_client *client, struct d6t_t *d6t,
			    u8 reg, u8 value);

static inline u32 d6t_checkPEC(struct i2c_client *client, struct d6t_t *d6t);

static inline u32 calc_crc(uint8_t data);

static inline s16 conv8us_s16_le(uint8_t *buf, int n);

#endif /* _D6T_CORE_H */