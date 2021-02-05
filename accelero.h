#ifndef ACCELERO_H_
#define ACCELERO_H_



// ---------- Sampling ----------

enum accelero_data_rate {
	ACCELERO_DATA_RATE_POWER_DOWN,
	ACCELERO_DATA_RATE_1_HZ,
	ACCELERO_DATA_RATE_10_HZ,
	ACCELERO_DATA_RATE_25_HZ,
	ACCELERO_DATA_RATE_50_HZ,
	ACCELERO_DATA_RATE_100_HZ,
	ACCELERO_DATA_RATE_200_HZ,
	ACCELERO_DATA_RATE_400_HZ,
	ACCELERO_DATA_RATE_1620_HZ,
	ACCELERO_DATA_RATE_5376_HZ,
	ACCELERO_DATA_RATE_MIN = ACCELERO_DATA_RATE_1_HZ,
	ACCELERO_DATA_RATE_MAX = ACCELERO_DATA_RATE_5376_HZ, //< does not work properly !
};


// ---------- Channels ----------

#define ACC_NB_CHAN 3

#define ACC_SCAN_X 0
#define ACC_SCAN_Y 1
#define ACC_SCAN_Z 2

#define ACC_LSM_CHANNEL(index, ch2, addr) \
{ \
	.scan_index = index, \
	.channel2 = ch2, \
	.address = addr, \
	.type = IIO_ACCEL, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE) | BIT(IIO_CHAN_INFO_SAMP_FREQ) | BIT(IIO_CHAN_INFO_SCALE), \
	.modified = 1, \
	.scan_type = { \
		.sign = 's', \
		.endianness = IIO_LE, \
		.realbits = 8, \
		.storagebits = 8, \
		.shift = 0, \
	}, \
}


// ---------- Registers ----------

#define REG_WHO_AM_I        0x0F //< R    Device Identification (0x33)

#define REG_CTRL_REG1       0x20 //< R/W  Control register (LPen & Data rate selection)
#define REG_CTRL_REG3       0x22 //< R/W  Control register (FIFO watermark -> INT1 pin)
#define REG_CTRL_REG4       0x23 //< R/W  Control register (Full scale selection)
#define REG_CTRL_REG5       0x24 //< R/W  Control register (FIFO enable)
#define REG_CTRL_REG6       0x25 //< R/W  Control register (INTx polarity -> INTx active-low)

#define REG_FIFO_READ_START 0x28 //< R    Acceleration data
#define REG_OUT_X_H         0x29 //< R    X-axis acceleration data
#define REG_OUT_Y_H         0x2B //< R    Y-axis acceleration data
#define REG_OUT_Z_H         0x2D //< R    Z-axis acceleration data

#define REG_FIFO_CTRL_REG   0x2E //< R/W  FIFO Control register (Stream mode & Watermark)

#define REG_INT1_SRC        0x31 //< R    Interrupt 1 Source register
#define REG_INT1_DURATION   0x33 //< R/W  Interrupt 1 Duration register



#endif /* ACCELERO_H_ */

