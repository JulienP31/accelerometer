#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/interrupt.h>
#include "accelero.h"


#define DEBUG


// struct accelero_dev [NOTA : private driver structure]
struct accelero_dev {
	struct i2c_client *i2c_client;
	bool enabled;
	enum accelero_data_rate rate;
};


// Constants
#define ACCELERO_NB_POINTS 4 //< max 16 (acc fifo size = 32)

static const u8 accelero_fullscale = 0; //< +/-2g
static const u8 accelero_lsb[] = {16, 32, 62, 186}; //< Unit : mg/LSB


// accelero_read_register()
static inline int accelero_read_register(struct i2c_client *client, u8 reg, u8 *byte)
{
	int ret = 0;
	
	ret = i2c_smbus_read_byte_data(client, reg); //< [NOTA : I2C bus locked]
	if (ret < 0)
	{
		return ret;
	}
	*byte = ret;
	
	return 0;
}


// accelero_write_register()
static inline int accelero_write_register(struct i2c_client *client, u8 reg, u8 byte)
{
	return i2c_smbus_write_byte_data(client, reg, byte); //< [NOTA : I2C bus locked]
}


// accelero_stop()
static int accelero_stop(struct i2c_client *client)
{
	int ret = 0;
	
	// Power-down
	ret = accelero_write_register(client, REG_CTRL_REG1, 0x08);
	if (ret < 0)
		return ret;
	
	// Bypass mode to reset FIFO
	ret = accelero_write_register(client, REG_FIFO_CTRL_REG, 0);
	if (ret < 0)
		return ret;
	
	return 0;
}


// accelero_start()
static int accelero_start(struct i2c_client *client, enum accelero_data_rate rate, u8 nbSamp)
{
	int ret = 0;

	// Reset
	ret = accelero_stop(client);
	if (ret < 0)
		return ret;
	
	// LPen
	ret = accelero_write_register(client, REG_CTRL_REG1, 1<<3);
	if (ret < 0)
		return ret;
	
	// FIFO watermark -> INT1 pin
	ret = accelero_write_register(client, REG_CTRL_REG3, 1<<2);
	if (ret < 0)
		return ret;
	
	// Full scale selection
	ret = accelero_write_register(client, REG_CTRL_REG4, accelero_fullscale<<4);
	if (ret < 0)
		return ret;
	
	// FIFO enable
	ret = accelero_write_register(client, REG_CTRL_REG5, 1<<6);
	if (ret < 0)
		return ret;
	
	// INTx polarity -> INTx active-low
	ret = accelero_write_register(client, REG_CTRL_REG6, 1<<1);
	if (ret < 0)
		return ret;
	
	// Interrupt 1 Duration register
	ret = accelero_write_register(client, REG_INT1_DURATION, 1);
	if (ret < 0)
		return ret;
	
	// Stream mode & Watermark
	ret = accelero_write_register(client, REG_FIFO_CTRL_REG, 2<<6 | ((nbSamp-1) & 0x1F));
	if (ret < 0)
		return ret;
	
	// Data rate selection -> Start acquisition
	ret = accelero_write_register(client, REG_CTRL_REG1, ((u8)rate)<<4 | 0x0F);
	if (ret < 0)
		return ret;
	
	return 0;
}


// accelero_get_sample()
static int accelero_get_sample(struct i2c_client *client, s8 *x, s8 *y, s8 *z)
{
	u8 byte = 0;
	int ret = 0;
	
	// Trigger reading (with b7 = 1 to enable auto-increment)
	ret = accelero_read_register(client, REG_FIFO_READ_START | 1 << 7, &byte);
	if (ret < 0)
		return ret;
	
	// Read XYZ
	ret = accelero_read_register(client, REG_OUT_X_H, x);
	if (ret < 0)
		return ret;
	
	ret = accelero_read_register(client, REG_OUT_Y_H, y);
	if (ret < 0)
		return ret;
	
	ret = accelero_read_register(client, REG_OUT_Z_H, z);
	if (ret < 0)
		return ret;
	
	return 0;
}

	
// accelero_irq_handler() -> threaded i.e. bottom-half [NOTA : needed because of blocking I2C communication]
irqreturn_t accelero_irq_handler(int irq, void *dev_id)
{
	struct i2c_client *client = (struct i2c_client *)dev_id;
	struct iio_dev *indio_dev = NULL;
	u8 byte = 0;
	s8 buf[ACC_NB_CHAN] = {0};
	int i = 0;
	int ret = 0;
	
	// Acknowledge IRQ
	ret = accelero_read_register(client, REG_INT1_SRC, &byte);
	if (ret < 0)
	{
		dev_err(&client->dev, "Could not acknowledge INT1 IRQ");
	}
	
	// Get data
	indio_dev = dev_get_drvdata(&client->dev);
	
	for (i = 0 ; ret == 0 && i < ACCELERO_NB_POINTS ; i++)
	{
		// Get signed raw data
		ret = accelero_get_sample(client, buf+0, buf+1, buf+2);

		// Push data to buffer
		iio_push_to_buffers(indio_dev, buf); //< [NOTA : some kind of wake_up_interruptible function is called here]
	}
	
	if (ret < 0)
	{
		dev_dbg(&client->dev, "Could not get data");
	}
	
	return IRQ_HANDLED;
}


// ---------- accelero_read_raw() ----------
static int accelero_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct accelero_dev *accelero = iio_priv(indio_dev);
	
	switch (mask)
	{
		case IIO_CHAN_INFO_ENABLE:
			*val = accelero->enabled;
			return IIO_VAL_INT;
		case IIO_CHAN_INFO_SAMP_FREQ:
			*val = accelero->rate;
			return IIO_VAL_INT;
		case IIO_CHAN_INFO_SCALE:
			*val = accelero_lsb[accelero_fullscale];
			return IIO_VAL_INT;
	}
	
	return -EINVAL;
}


// ---------- accelero_write_raw() ----------
static int accelero_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct accelero_dev *accelero = iio_priv(indio_dev);
	int ret = 0;
	
	switch (mask)
	{
		case IIO_CHAN_INFO_ENABLE:
			if (val == 1)
			{		
				// Start accelero
				ret = accelero_start(accelero->i2c_client, accelero->rate, ACCELERO_NB_POINTS);
				if (ret < 0)
				{
					dev_err(&accelero->i2c_client->dev, "Could not start accelero");
					return ret;
				}
				accelero->enabled = true;
			}
			else if (val == 0)
			{
				// Stop accelero
				ret = accelero_stop(accelero->i2c_client);
				if (ret < 0)
				{
					dev_err(&accelero->i2c_client->dev, "Could not stop accelero");
					return ret;
				}
				accelero->enabled = false;
			}
			else
			{
				return -EINVAL;
			}
			
			return 0;
		case IIO_CHAN_INFO_SAMP_FREQ:
			// Update rate
			if (val >= ACCELERO_DATA_RATE_MIN && val <= ACCELERO_DATA_RATE_MAX)
				accelero->rate = val;
			else
				return -EINVAL;
			
			// Restart accelero
			if (accelero->enabled)
			{
				// Stop
				accelero_stop(accelero->i2c_client);
				
				// Flush buffer [NOTA : accelero stopped -> INT1 not triggered -> ...
							// ...no need to protect buffer with spin_lock_bh and so on]
				/* TODO */
				/* iio_push_to_buffers_with_timestamp could be used in INT1 IRQ handler as a workaround */
				
				// Re-start
				ret = accelero_start(accelero->i2c_client, accelero->rate, ACCELERO_NB_POINTS);
				if (ret < 0)
				{
					dev_err(&accelero->i2c_client->dev, "Could not restart accelero");
					return ret;
				}				
			}
			
			return 0;
	}
	
	return -EINVAL;
}


// ---------- struct accelero_info ----------
static const struct iio_info accelero_info = {
	.read_raw = accelero_read_raw,
	.write_raw = accelero_write_raw,
};


// ---------- accelero_channels[] ----------
static const struct iio_chan_spec accelero_channels[] = {
	ACC_LSM_CHANNEL(ACC_SCAN_X, IIO_MOD_X, REG_OUT_X_H),
	ACC_LSM_CHANNEL(ACC_SCAN_Y, IIO_MOD_Y, REG_OUT_Y_H),
	ACC_LSM_CHANNEL(ACC_SCAN_Z, IIO_MOD_Z, REG_OUT_Z_H),
};


// ---------- accelero_probe() ----------
static int accelero_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct accelero_dev *accelero = NULL;
	struct iio_dev *indio_dev = NULL;
	struct iio_buffer *indio_buf = NULL;
	u8 acc_id = 0;
	int ret = 0;
	
	// Allocate memory for IIO device (including private data)
	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*accelero));
	if (!indio_dev)
	{
		dev_err(&client->dev, "Could not allocate iio device structure");
		return -ENOMEM;
	}
	
	// Initialize private driver infos
	accelero = iio_priv(indio_dev); //< [NOTA : accelero_dev from iio_dev]
	accelero->i2c_client = client;  //< [NOTA : i2c_client from accelero_dev]
	accelero->enabled = false;
	accelero->rate = ACCELERO_DATA_RATE_MIN;
	
	dev_set_drvdata(&client->dev, indio_dev); //< [NOTA : iio_dev (logical device handled by iio subsystem)...
							// ...from i2c_client (physical device)]
	
	// Initialize IIO device fields with driver specific information
	indio_dev->name = devm_kasprintf(&client->dev, GFP_KERNEL, "accelero-%x", client->addr);
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &accelero_info;
	indio_dev->modes = INDIO_BUFFER_SOFTWARE;
	indio_dev->channels = accelero_channels;
	indio_dev->num_channels = ARRAY_SIZE(accelero_channels);
	
	// Setup data buffer
	indio_buf = devm_iio_kfifo_allocate(&client->dev);
	if (!indio_buf)
	{
		dev_err(&client->dev, "Could not allocate iio buffer");
		return -ENOMEM;
	}
	
	iio_device_attach_buffer(indio_dev, indio_buf);
	
	// Register device with IIO subsystem
	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret < 0)
	{
		dev_err(&client->dev, "Could not register iio device");
		return ret;
	}
	
	// Request accelero INT1 pin IRQ
	ret = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, accelero_irq_handler, IRQF_ONESHOT,
					"accelero_irq", client);
	if (ret < 0)
	{
		dev_err(&client->dev, "Could not request INT1 IRQ");
		return ret;
	}	

	// Check accelero ID
	ret = accelero_read_register(client, REG_WHO_AM_I, &acc_id);
	if (ret < 0)
	{
		dev_err(&client->dev, "Could not read accelero ID");
		return ret;
	}
	dev_info(&client->dev, "acc_id = 0x%x", acc_id);
	
	return 0;
}


// ---------- accelero_remove() ----------
static int accelero_remove(struct i2c_client *client)
{
	// Stop accelero
	accelero_stop(client);
	
	return 0;
}


// ---------- accelero_of_match[] ----------
static const struct of_device_id accelero_of_match[] = {
	{ .compatible = "st,accelero-lis2de12" },
	{ },
};
MODULE_DEVICE_TABLE(of, accelero_of_match);


// ---------- struct accelero_i2c_driver ----------
static struct i2c_driver accelero_i2c_driver = {
	.driver = {
		.name = "accelero_i2c",
		.of_match_table = accelero_of_match,
	},
	.probe = accelero_probe,
	.remove = accelero_remove,
};
module_i2c_driver(accelero_i2c_driver);


MODULE_DESCRIPTION("LIS2DE12 accelerometer");
MODULE_AUTHOR("Julien Panis <julienpanis@hotmail.com>");
MODULE_LICENSE("GPL v2");

