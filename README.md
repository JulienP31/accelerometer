# accelerometer
LIS2DE12 accelerometer I2C driver



1) Connections with BeagleBone Black Wireless

GND  -> 1
SD0  -> 2  (I2C address LSB = 0)
VDD  -> 3  (idem VDD_IO)
CS   -> 4  (I2C/SPI = 1/0)
INT1 -> 15
SCL  -> 17
SDA  -> 18



2) User space

insmod accelero.ko

echo 1 > /sys/bus/iio/devices/iio\:device0/scan_elements/in_accel_x_en
echo 1 > /sys/bus/iio/devices/iio\:device0/scan_elements/in_accel_y_en
echo 1 > /sys/bus/iio/devices/iio\:device0/scan_elements/in_accel_z_en
echo 100 > /sys/bus/iio/devices/iio\:device0/buffer/length
echo 1 > /sys/bus/iio/devices/iio\:device0/buffer/watermark
echo 1 > /sys/bus/iio/devices/iio\:device0/buffer/enable

echo 2 > /sys/bus/iio/devices/iio\:device0/sampling_frequency
echo 1 > /sys/bus/iio/devices/iio\:device0/en

cat /sys/bus/iio/devices/iio\:device0/scale

cat /dev/iio\:device0 | hexdump -v -e'3/1 "%d ""\n"'
	[NOTA about hexdump : data NOT signed despite %d !]

rmmod accelero.ko

