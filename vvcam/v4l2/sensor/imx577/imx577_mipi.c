#include <asm/unaligned.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/of_graph.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ioctl.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include "vvsensor.h"

#include "imx577_regs_common.h"
#include "imx577_regs_4k.h"

#define IMX577_REG_ID		0x0016
#define IMX577_ID		0x577
#define IMX577_RESERVE_ID	0x577

#define IMX577_REG_MODE_SELECT  0x0100
#define IMX577_MODE_STANDBY     0x00
#define IMX577_MODE_STREAMING   0x01

#define IMX577_REG_EXPOSURE_CIT 0x0202
#define IMX577_EXPOSURE_MIN     8
#define IMX577_EXPOSURE_OFFSET  22
#define IMX577_EXPOSURE_STEP    1
#define IMX577_EXPOSURE_DEFAULT 0x0648

#define IMX577_REG_AGAIN        0x0204
#define IMX577_AGAIN_MIN        0
#define IMX577_AGAIN_MAX        978
#define IMX577_AGAIN_STEP       1
#define IMX577_AGAIN_DEFAULT    0

#define IMX577_REG_DGAIN_GLOBAL	0x3ff9
#define IMX577_REG_DGAIN_GR	0x020e
#define IMX577_REG_DGAIN_R	0x0210
#define IMX577_REG_DGAIN_B	0x0212
#define IMX577_REG_DGAIN_GB	0x0214

#define IMX577_REG_HOLD         0x0104

#define IMX577_REG_LPFR         0x0340

#define IMX577_REG_FLASH_DELAY	0x0c14

#define IMX577_SENS_PAD_SOURCE	0
#define IMX577_SENS_PADS_NUM	1

#define client_to_imx577(client)\
	container_of(i2c_get_clientdata(client), struct imx577, subdev)

struct imx577_capture_properties {
	__u64 max_lane_frequency;
	__u64 max_pixel_frequency;
	__u64 max_data_rate;
};

struct imx577 {
	struct device *dev;
	struct i2c_client *i2c_client;
	struct clk *inclk;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *shutter_gpio;
	struct imx577_capture_properties icp;

	struct v4l2_subdev subdev;
	struct media_pad pads[IMX577_SENS_PADS_NUM];

	struct v4l2_mbus_framefmt format;
	vvcam_mode_info_t cur_mode;
	sensor_blc_t blc;
	sensor_white_balance_t wb;
	struct mutex lock;
	u32 stream_status;
	u32 resume_status;
};

static struct vvcam_mode_info_s pimx577_mode_info[] = {
	{
		.index		= 0,
		.size		= {
			.bounds_width	= 4048,
			.bounds_height	= 3040,
			.top		= 0,
			.left		= 0,
			.width		= 4048,
			.height		= 3040,
		},
		.hdr_mode	= SENSOR_MODE_LINEAR,
		.bit_width	= 10,
		.data_compress	= { .enable = 0, },
		.bayer_pattern	= BAYER_RGGB,
		.ae_info	= {
			.def_frm_len_lines	= 0x0c1e,
			.curr_frm_len_lines	= 0x0c1e,
			.one_line_exp_time_ns	= 9400,
			.max_integration_line	= 0x0c1e - 22,
			.min_integration_line	= 8,

			.max_again		= 22.261 * (1 << SENSOR_FIX_FRACBITS),
			.min_again		= 1 * (1 << SENSOR_FIX_FRACBITS),
			.max_dgain		= 1 * (1 << SENSOR_FIX_FRACBITS),
			.min_dgain		= 1 * (1 << SENSOR_FIX_FRACBITS),

			.start_exposure		= 1 * 0x0648 * (1 << SENSOR_FIX_FRACBITS),
			.gain_step		= 1 * (1 << SENSOR_FIX_FRACBITS),
			.cur_fps		= 34.29 * (1 << SENSOR_FIX_FRACBITS),
			.max_fps		= 34.31 * (1 << SENSOR_FIX_FRACBITS),
			.min_fps		= 1 * (1 << SENSOR_FIX_FRACBITS),
			.min_afps		= 5 * (1 << SENSOR_FIX_FRACBITS),
			.int_update_delay_frm	= 1,
			.gain_update_delay_frm	= 1,
		},
		.mipi_info 	= {
			.mipi_lane = 4,
		},
		.preg_data	= imx577_init_setting_4k,
		.reg_data_count	= ARRAY_SIZE(imx577_init_setting_4k),
	},
};

static int imx577_write_register(struct imx577 *sensor, u16 reg, u32 len, u32 val)
{
	u8 buf[6] = {0};

	put_unaligned_be16(reg, buf);
	put_unaligned_be32(val << (8 * (4 - len)), buf + 2);
	if (i2c_master_send(sensor->i2c_client, buf, len + 2) != len + 2) {
		dev_err(sensor->dev, "Write reg error: reg=%x, val=%x\n", reg, val);
		return -EIO;
	}

	return 0;
}

static int imx577_write_reg(struct imx577 *sensor, u16 reg, u8 val)
{
	return imx577_write_register(sensor, reg, 1, val);
}

static int imx577_write_reg16(struct imx577 *sensor, u16 reg, u16 val)
{
	return imx577_write_register(sensor, reg, 2, val);
}

static int imx577_write_reg_array(struct imx577 *sensor,
				  struct vvcam_sccb_data_s *reg_arry,
				  u32 size)
{
	int i = 0;
	int ret = 0;
	struct i2c_msg msg;
	u8 *send_buf;
	u32 send_buf_len = 0;
	struct i2c_client *i2c_client = sensor->i2c_client;

	send_buf = (u8 *)kmalloc(size + 2, GFP_KERNEL);
	if (!send_buf)
		return -ENOMEM;

	send_buf[send_buf_len++] = (reg_arry[0].addr >> 8) & 0xff;
	send_buf[send_buf_len++] = reg_arry[0].addr & 0xff;
	send_buf[send_buf_len++] = reg_arry[0].data & 0xff;
	for (i=1; i < size; i++) {
		if (reg_arry[i].addr == (reg_arry[i-1].addr + 1)){
			send_buf[send_buf_len++] = reg_arry[i].data & 0xff;
		} else {
			msg.addr  = i2c_client->addr;
			msg.flags = i2c_client->flags;
			msg.buf   = send_buf;
			msg.len   = send_buf_len;
			ret = i2c_transfer(i2c_client->adapter, &msg, 1);
			if (ret < 0) {
				dev_err(sensor->dev, "%s: i2c transfer error\n", __func__);
				kfree(send_buf);
				return ret;
			}
			send_buf_len = 0;
			send_buf[send_buf_len++] =
				(reg_arry[i].addr >> 8) & 0xff;
			send_buf[send_buf_len++] =
				reg_arry[i].addr & 0xff;
			send_buf[send_buf_len++] =
				reg_arry[i].data & 0xff;
		}
	}

	if (send_buf_len > 0) {
		msg.addr  = i2c_client->addr;
		msg.flags = i2c_client->flags;
		msg.buf   = send_buf;
		msg.len   = send_buf_len;
		ret = i2c_transfer(i2c_client->adapter, &msg, 1);
		if (ret < 0)
			dev_err(sensor->dev, "%s: i2c transfer end meg error\n", __func__);
		else
			ret = 0;

	}
	kfree(send_buf);
	return ret;
}

static int imx577_read_reg(struct imx577 *sensor, u16 reg, u32 *val)
{
	const u32 len = 2;
	struct i2c_client *client = sensor->i2c_client;
	struct i2c_msg msgs[2] = {0};
	u8 addr_buf[2] = {0};
	u8 data_buf[4] = {0};
	int ret = 0;

	put_unaligned_be16(reg, addr_buf);

	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_buf[4 - len];

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ARRAY_SIZE(msgs) != ret)
		return -EIO;

	*val = get_unaligned_be32(data_buf);

	return 0;
}

static int imx577_power_on(struct imx577 *sensor)
{
	int ret = 0;

	ret = clk_prepare_enable(sensor->inclk);
	if (0 > ret) {
		dev_err(sensor->dev, "Failed to enable inclk");
		return ret;
	}

	usleep_range(5000, 5200);

	gpiod_set_value_cansleep(sensor->reset_gpio, 1);

	usleep_range(5000, 5200);

	return ret;
}

static int imx577_power_off(struct imx577 *sensor)
{
	gpiod_set_value_cansleep(sensor->reset_gpio, 0);

	clk_disable_unprepare(sensor->inclk);

	return 0;
}

static int imx577_detect(struct imx577 *sensor)
{
	int retries = 0, ret = 0;
	u32 val;

	do {
		ret = imx577_read_reg(sensor, IMX577_REG_ID, &val);
		if (-EIO == ret || IMX577_ID != val) {
			if (-EIO == ret) {
				dev_info(sensor->dev, "Got EIO looking for sensor id, retrying?");
			} else {
				dev_info(sensor->dev, "Expected %#x, got %#x, retrying?", IMX577_ID, val);
			}
			usleep_range(500, 1000);
		}
	} while ((-EIO == ret || IMX577_ID != val) && 10 > retries++);
	if (0 > ret)
		return ret;

	if (IMX577_ID != val) {
		dev_err(sensor->dev, "chip id mismatch: %x!=%x",
				IMX577_ID, val);
		return -ENXIO;
	}
	return 0;
}

static int imx577_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx577 *sensor = client_to_imx577(client);

	if (on) {
		imx577_power_on(sensor);
	} else {
		imx577_power_off(sensor);
	}

	return 0;
}

static int imx577_get_clk(struct imx577 *sensor, void *clk)
{
	int ret = 0;
	struct vvcam_clk_s vvcam_clk;

	vvcam_clk.sensor_mclk = clk_get_rate(sensor->inclk);
	vvcam_clk.csi_max_pixel_clk = sensor->icp.max_pixel_frequency;
	ret = copy_to_user(clk, &vvcam_clk, sizeof(struct vvcam_clk_s));
	if (0 != ret)
		ret = -EINVAL;

	return ret;
}

static int imx577_query_capability(struct imx577 *sensor, void *arg)
{
	struct v4l2_capability *pcap = (struct v4l2_capability *)arg;

	strcpy((char *)pcap->driver, "imx577");
	strcpy((char *)pcap->card, "Sony IMX577 Camera");
	strcpy((char *)pcap->bus_info, "csi0");

	if (sensor->i2c_client->adapter) {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] =
			(__u8)sensor->i2c_client->adapter->nr;
	} else {
		pcap->bus_info[VVCAM_CAP_BUS_INFO_I2C_ADAPTER_NR_POS] = 0xFF;
	}

	return 0;
}

static int imx577_query_supports(struct imx577 *sensor, void *parray)
{
	int ret = 0;
	struct vvcam_mode_info_array_s *psensor_mode_array = parray;

	psensor_mode_array->count = ARRAY_SIZE(pimx577_mode_info);
	ret = copy_to_user(&psensor_mode_array->modes, pimx577_mode_info,
			   sizeof(pimx577_mode_info));

	if (0 != ret)
		ret = -ENOMEM;
	return ret;
}

static int imx577_get_sensor_id(struct imx577 *sensor, void *pchip_id)
{
	u32 chip_id = 0;
	u32 val = 0;
	int ret = imx577_read_reg(sensor, IMX577_REG_ID, &val);
	if (0 != ret)
		return ret;

	chip_id = val;
	ret = copy_to_user(pchip_id, &chip_id, sizeof(u32));
	if (0 != ret)
		ret = -ENOMEM;
	return ret;
}

static int imx577_get_reserve_id(struct imx577 *sensor, void *preserve_id)
{
	u32 reserve_id = IMX577_RESERVE_ID;
	int ret = copy_to_user(preserve_id, &reserve_id, sizeof(u32));
	if (0 != ret)
		ret = -ENOMEM;
	return ret;
}

static int imx577_get_sensor_mode(struct imx577 *sensor, void *pmode)
{
	int ret = copy_to_user(pmode, &sensor->cur_mode,
			sizeof(struct vvcam_mode_info_s));
	if (0 != ret)
		ret = -ENOMEM;
	return ret;
}

static int imx577_set_sensor_mode(struct imx577 *sensor, void *pmode)
{
	int ret = 0;
	int i = 0;
	struct vvcam_mode_info_s sensor_mode;

	ret = copy_from_user(&sensor_mode, pmode,
		sizeof(struct vvcam_mode_info_s));
	if (0 != ret)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(pimx577_mode_info); i++) {
		if (pimx577_mode_info[i].index == sensor_mode.index) {
			memcpy(&sensor->cur_mode, &pimx577_mode_info[i],
				sizeof(struct vvcam_mode_info_s));
			return 0;
		}
	}

	return -ENXIO;
}

static int imx577_set_exp(struct imx577 *sensor, u32 exp)
{
	int ret = 0;
	u32 integration = exp & 0xffff;
	ret = imx577_write_reg(sensor, IMX577_REG_HOLD, 1);
	if (ret)
		return ret;

	ret = imx577_write_reg16(sensor, IMX577_REG_EXPOSURE_CIT, integration);
	imx577_write_reg(sensor, IMX577_REG_HOLD, 0);

	return ret;
}

static int imx577_set_vsexp(struct imx577 *sensor, u32 exp)
{
	int ret = 0;

	// TODO: implement

	return ret;
}

static int imx577_set_gain(struct imx577 *sensor, u32 total_gain)
{
	int ret = 0;
	const u32 convert_gain = 1 << SENSOR_FIX_FRACBITS;
	const u32 max_again = 22.261 * convert_gain;
	u16 again = 0, dgain = 0x0100;

	if (total_gain < convert_gain) {
		/* Just use the minimum default, we're below the min anyway */
	} else if (total_gain < max_again) {
		again = 1024 - 1024 * convert_gain / total_gain;
	} else {
		again = 978;
	}

	ret = imx577_write_reg(sensor, IMX577_REG_HOLD, 1);
	if (ret)
		return ret;

	ret = imx577_write_reg16(sensor, IMX577_REG_AGAIN, again);
	ret |= imx577_write_reg16(sensor, IMX577_REG_DGAIN_GR, dgain);
	imx577_write_reg(sensor, IMX577_REG_HOLD, 0);

	return ret;
}

static int imx577_set_vsgain(struct imx577 *sensor, u32 total_gain)
{
	return imx577_set_gain(sensor, total_gain);
}

static int imx577_set_fps(struct imx577 *sensor, u32 fps)
{
	int ret = 0;
	u32 frm_len_lines = sensor->cur_mode.ae_info.curr_frm_len_lines;
	const u32 pixel_rate = 120000000 * 4;
	const u32 line_length_pck = 4512;
	const u8 fll_lshift = 0;
	u32 flash_delay = 0;
	
	if (fps > sensor->cur_mode.ae_info.max_fps) {
		fps = sensor->cur_mode.ae_info.max_fps;
	} else if (fps < sensor->cur_mode.ae_info.min_fps) {
		fps = sensor->cur_mode.ae_info.min_fps;
	}

	frm_len_lines = pixel_rate / ((fps * line_length_pck * (1 << fll_lshift)) >> SENSOR_FIX_FRACBITS);


	ret = imx577_write_reg16(sensor, IMX577_REG_LPFR, frm_len_lines);
	if (ret)
		return ret;

	flash_delay = frm_len_lines - (sensor->cur_mode.size.height / 2);
	ret = imx577_write_reg16(sensor, IMX577_REG_FLASH_DELAY, flash_delay);
	if (ret)
		return ret;
	
	sensor->cur_mode.ae_info.curr_frm_len_lines = frm_len_lines;
	sensor->cur_mode.ae_info.max_integration_line = frm_len_lines - 22;
	fps = (pixel_rate / (frm_len_lines * (1 << fll_lshift) * line_length_pck)) << SENSOR_FIX_FRACBITS;
	sensor->cur_mode.ae_info.cur_fps = fps;

	return ret;
}

static int imx577_get_fps(struct imx577 *sensor, u32 *pfps)
{
	*pfps = sensor->cur_mode.ae_info.cur_fps;
	return 0;
}

static int imx577_set_ratio(struct imx577 *sensor, void* pratio)
{
	int ret = 0;
	struct sensor_hdr_artio_s hdr_ratio;
	struct vvcam_ae_info_s *pae_info = &sensor->cur_mode.ae_info;

	ret = copy_from_user(&hdr_ratio, pratio, sizeof(hdr_ratio));

	if (hdr_ratio.ratio_s_vs != pae_info->hdr_ratio.ratio_s_vs) {
		pae_info->hdr_ratio.ratio_s_vs = hdr_ratio.ratio_s_vs;
		if (sensor->cur_mode.hdr_mode != SENSOR_MODE_LINEAR) {
			sensor->cur_mode.ae_info.max_integration_line =
				pae_info->curr_frm_len_lines - pae_info->max_vsintegration_line - 8;
		}
	}

	return 0;
}

static int imx577_set_test_pattern(struct imx577 *sensor, void *arg)
{
	int ret = 0;
	struct sensor_test_pattern_s test_pattern;

	ret = copy_from_user(&test_pattern, arg, sizeof(test_pattern));
	if (0 != ret)
		return -ENOMEM;

	// TODO: set test pattern on the sensor.

	return ret;
}

static int imx577_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx577 *sensor = client_to_imx577(client);

	if (enable)
		imx577_write_reg(sensor, IMX577_REG_MODE_SELECT, IMX577_MODE_STREAMING);
	else
		imx577_write_reg(sensor, IMX577_REG_MODE_SELECT, IMX577_MODE_STANDBY);

	sensor->stream_status = enable;
	return 0;
}

static int imx577_get_format_code(struct imx577 *sensor, u32 *code)
{
	switch (sensor->cur_mode.bayer_pattern) {
	case BAYER_RGGB:
		if (sensor->cur_mode.bit_width == 8) {
			*code = MEDIA_BUS_FMT_SRGGB8_1X8;
		} else if (sensor->cur_mode.bit_width == 10) {
			*code = MEDIA_BUS_FMT_SRGGB10_1X10;
		} else {
			*code = MEDIA_BUS_FMT_SRGGB12_1X12;
		}
		break;
	case BAYER_GRBG:
		if (sensor->cur_mode.bit_width == 8) {
			*code = MEDIA_BUS_FMT_SGRBG8_1X8;
		} else if (sensor->cur_mode.bit_width == 10) {
			*code = MEDIA_BUS_FMT_SGRBG10_1X10;
		} else {
			*code = MEDIA_BUS_FMT_SGRBG12_1X12;
		}
		break;
	case BAYER_GBRG:
		if (sensor->cur_mode.bit_width == 8) {
			*code = MEDIA_BUS_FMT_SGBRG8_1X8;
		} else if (sensor->cur_mode.bit_width == 10) {
			*code = MEDIA_BUS_FMT_SGBRG10_1X10;
		} else {
			*code = MEDIA_BUS_FMT_SGBRG12_1X12;
		}
		break;
	case BAYER_BGGR:
		if (sensor->cur_mode.bit_width == 8) {
			*code = MEDIA_BUS_FMT_SBGGR8_1X8;
		} else if (sensor->cur_mode.bit_width == 10) {
			*code = MEDIA_BUS_FMT_SBGGR10_1X10;
		} else {
			*code = MEDIA_BUS_FMT_SBGGR12_1X12;
		}
		break;
	default:
		/*nothing need to do*/
		break;
	}
	return 0;
}

static void imx577_fill_pad_format(struct imx577 *sensor,
				   const struct vvcam_mode_info_s *mode,
				   struct v4l2_subdev_format *fmt)
{
	u32 cur_code = MEDIA_BUS_FMT_SBGGR12_1X12;
	imx577_get_format_code(sensor, &cur_code);

	fmt->format.width = mode->size.width;
	fmt->format.height = mode->size.height;
	fmt->format.code = cur_code;
	fmt->format.field = V4L2_FIELD_NONE;
	fmt->format.colorspace = V4L2_COLORSPACE_RAW;
	fmt->format.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->format.quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->format.xfer_func = V4L2_XFER_FUNC_NONE;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int imx577_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_mbus_code_enum *code)
#else
static int imx577_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
#endif
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx577 *sensor = client_to_imx577(client);

	u32 cur_code = MEDIA_BUS_FMT_SBGGR12_1X12;

	if (code->index > 0)
		return -EINVAL;
	imx577_get_format_code(sensor, &cur_code);
	code->code = cur_code;

	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int imx577_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fsize)
#else
static int imx577_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fsize)
#endif
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx577 *sensor = client_to_imx577(client);

	fsize->min_width = sensor->cur_mode.size.width;
	fsize->max_width = sensor->cur_mode.size.width;
	fsize->min_height = sensor->cur_mode.size.height;
	fsize->max_height = sensor->cur_mode.size.height;

	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int imx577_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
#else
static int imx577_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
#endif
{
	int ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx577 *sensor = client_to_imx577(client);
	mutex_lock(&sensor->lock);

	if ((fmt->format.width != sensor->cur_mode.size.bounds_width) ||
	    (fmt->format.height != sensor->cur_mode.size.bounds_height)) {
		dev_err(sensor->dev, "%s: set sensor format %dx%d error\n",
			__func__, fmt->format.width, fmt->format.height);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	ret = imx577_write_reg_array(sensor,
		(struct vvcam_sccb_data_s *)sensor->cur_mode.preg_data,
		sensor->cur_mode.reg_data_count);
	if (ret < 0) {
		dev_err(sensor->dev, "%s: imx577_write_reg_array error\n", __func__);
		mutex_unlock(&sensor->lock);
		return -EINVAL;
	}

	imx577_get_format_code(sensor, &fmt->format.code);
	fmt->format.field = V4L2_FIELD_NONE;
	sensor->format = fmt->format;
	mutex_unlock(&sensor->lock);
	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int imx577_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
#else
static int imx577_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
#endif
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx577 *sensor = client_to_imx577(client);

	mutex_lock(&sensor->lock);
	fmt->format = sensor->format;
	mutex_unlock(&sensor->lock);
	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
static int imx577_init_pad_cfg(struct v4l2_subdev *sd,
			       struct v4l2_subdev_state *state)
#else
static int imx577_init_pad_cfg(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg)
#endif
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx577 *sensor = client_to_imx577(client);
	struct v4l2_subdev_format fmt = { 0 };

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
	fmt.which = state ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
#else
	fmt.which = cfg ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
#endif

	imx577_fill_pad_format(sensor, &sensor->cur_mode, &fmt);
	return imx577_set_fmt(sd, state, &fmt);
}

static long imx577_priv_ioctl(struct v4l2_subdev *sd,
			      unsigned int cmd,
			      void *arg)
{
	long ret = 0;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct imx577 *sensor = client_to_imx577(client);
	struct vvcam_sccb_data_s sensor_reg;

	mutex_lock(&sensor->lock);
	switch (cmd){
	case VVSENSORIOC_S_POWER:
		ret = 0;
		break;
	case VVSENSORIOC_S_CLK:
		ret = 0;
		break;
	case VVSENSORIOC_G_CLK:
		ret = imx577_get_clk(sensor,arg);
		break;
	case VVSENSORIOC_RESET:
		ret = 0;
		break;
	case VIDIOC_QUERYCAP:
		ret = imx577_query_capability(sensor, arg);
		break;
	case VVSENSORIOC_QUERY:
		ret = imx577_query_supports(sensor, arg);
		break;
	case VVSENSORIOC_G_CHIP_ID:
		ret = imx577_get_sensor_id(sensor, arg);
		break;
	case VVSENSORIOC_G_RESERVE_ID:
		ret = imx577_get_reserve_id(sensor, arg);
		break;
	case VVSENSORIOC_G_SENSOR_MODE:
		ret = imx577_get_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_SENSOR_MODE:
		ret = imx577_set_sensor_mode(sensor, arg);
		break;
	case VVSENSORIOC_S_STREAM:
		ret = imx577_s_stream(&sensor->subdev, *(int *)arg);
		break;
	case VVSENSORIOC_WRITE_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= imx577_write_reg(sensor, sensor_reg.addr,
			sensor_reg.data);
		break;
	case VVSENSORIOC_READ_REG:
		ret = copy_from_user(&sensor_reg, arg,
			sizeof(struct vvcam_sccb_data_s));
		ret |= imx577_read_reg(sensor, sensor_reg.addr,
			&sensor_reg.data);
		ret |= copy_to_user(arg, &sensor_reg,
			sizeof(struct vvcam_sccb_data_s));
		break;
	case VVSENSORIOC_S_EXP:
		ret = imx577_set_exp(sensor, *(u32 *)arg);
		break;
	case VVSENSORIOC_S_VSEXP:
		ret = imx577_set_vsexp(sensor, *(u32 *)arg);
		break;
	case VVSENSORIOC_S_GAIN:
		ret = imx577_set_gain(sensor, *(u32 *)arg);
		break;
	case VVSENSORIOC_S_VSGAIN:
		ret = imx577_set_vsgain(sensor, *(u32 *)arg);
		break;
	case VVSENSORIOC_S_FPS:
		ret = imx577_set_fps(sensor, *(u32 *)arg);
		break;
	case VVSENSORIOC_G_FPS:
		ret = imx577_get_fps(sensor, (u32 *)arg);
		break;
	case VVSENSORIOC_S_HDR_RADIO:
		ret = imx577_set_ratio(sensor, arg);
		break;
	case VVSENSORIOC_S_TEST_PATTERN:
		ret= imx577_set_test_pattern(sensor, arg);
		break;
	default:
		v4l_printk_ioctl(NULL, cmd);
		dev_err(sensor->dev, "Unrecognized ioctl (%#x) received.", cmd);
		break;
	}

	mutex_unlock(&sensor->lock);
	return ret;
}

static struct v4l2_subdev_video_ops imx577_subdev_video_ops = {
	.s_stream = imx577_s_stream,
};

static const struct v4l2_subdev_pad_ops imx577_subdev_pad_ops = {
	.init_cfg = imx577_init_pad_cfg,
	.enum_mbus_code = imx577_enum_mbus_code,
	.enum_frame_size = imx577_enum_frame_size,
	.set_fmt = imx577_set_fmt,
	.get_fmt = imx577_get_fmt,
};

static const struct v4l2_subdev_core_ops imx577_subdev_core_ops = {
	.s_power = imx577_s_power,
	.ioctl = imx577_priv_ioctl,
};

static struct v4l2_subdev_ops imx577_subdev_ops = {
	.core = &imx577_subdev_core_ops,
	.video = &imx577_subdev_video_ops,
	.pad = &imx577_subdev_pad_ops,
};

static int imx577_link_setup(struct media_entity *entity,
		const struct media_pad *local,
		const struct media_pad *remote, u32 flags)
{
	return 0;
}

static const struct media_entity_operations imx577_sd_media_ops = {
	.link_setup = imx577_link_setup,
};

static int imx577_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret;
	struct device *dev = &client->dev;
	struct v4l2_subdev *sd;
	struct imx577 *sensor;

	sensor = devm_kmalloc(dev, sizeof(*sensor), GFP_KERNEL);
	if (!sensor)
		return -ENOMEM;
	memset(sensor, 0, sizeof(*sensor));

	sensor->dev = dev;
	sensor->i2c_client = client;

	sensor->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						     GPIOD_OUT_LOW);
	if (IS_ERR(sensor->reset_gpio)) {
		dev_err(dev, "failed to get reset gpio %ld",
			PTR_ERR(sensor->reset_gpio));
		return PTR_ERR(sensor->reset_gpio);
	}

	/* Get sensor input clock */
	sensor->inclk = devm_clk_get(dev, NULL);
	if (IS_ERR(sensor->inclk)) {
		dev_err(dev, "could not get inclk");
		return PTR_ERR(sensor->inclk);
	}

	/*
	rate = clk_get_rate(sensor->inclk);
	if (rate != IMX577_INCLK_RATE) {
		dev_err(dev, "inclk frequency mismatch");
		return -EINVAL;
	}
	*/

	ret = imx577_power_on(sensor);
	if (0 > ret) {
		dev_err(dev, "failed to power on the sensor");
		return ret;
	}

	ret = imx577_detect(sensor);
	if (0 > ret) {
		dev_err(dev, "failed to find sensor: %d", ret);
		goto probe_err_power_off;
	}

	ret = imx577_write_reg_array(sensor,
		(struct vvcam_sccb_data_s *)imx577_init_setting_common,
		ARRAY_SIZE(imx577_init_setting_common));
	if (ret < 0) {
		dev_err(sensor->dev, "%s: imx577_write_reg_array error\n", __func__);
		return -EINVAL;
	}

	/*
	 * Collecting the information about limits of capture path
	 * has been centralized to the sensor and the sensor endpoint.
	 */
	sensor->icp.max_lane_frequency = 600000000;
	sensor->icp.max_pixel_frequency = 480000000;
	sensor->icp.max_data_rate = 0;

	sd = &sensor->subdev;
	v4l2_i2c_subdev_init(sd, client, &imx577_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sd->dev = &client->dev;
	sd->entity.ops = &imx577_sd_media_ops;
	sd->entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->pads[IMX577_SENS_PAD_SOURCE].flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&sd->entity, IMX577_SENS_PADS_NUM,
			sensor->pads);

	if (0 > ret)
		goto probe_err_power_off;

#if LINUX_VERSION_CODE > KERNEL_VERSION(5, 12, 0)
	ret = v4l2_async_register_subdev_sensor(sd);
#else
	ret = v4l2_async_register_subdev_sensor_common(sd);
#endif
	if (ret < 0) {
		dev_err(dev, "%s--Async register failed, ret=%d\n",
			__func__, ret);
		goto probe_err_free_entity;
	}

	memcpy(&sensor->cur_mode, &pimx577_mode_info[0],
			sizeof(struct vvcam_mode_info_s));

	mutex_init(&sensor->lock);
	dev_info(sensor->dev, "%s camera mipi imx577, is found\n", __func__);

	return 0;

probe_err_free_entity:
	media_entity_cleanup(&sd->entity);

probe_err_power_off:
	imx577_power_off(sensor);

	return ret;
}

static int imx577_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct imx577 *sensor = client_to_imx577(client);

	v4l2_async_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	imx577_power_off(sensor);
	mutex_destroy(&sensor->lock);

	return 0;
}

static int __maybe_unused imx577_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct imx577 *sensor = client_to_imx577(client);

	sensor->resume_status = sensor->stream_status;
	if (sensor->resume_status) {
		imx577_s_stream(&sensor->subdev, 0);
	}

	return 0;
}

static int __maybe_unused imx577_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct imx577 *sensor = client_to_imx577(client);

	if (sensor->resume_status) {
		imx577_s_stream(&sensor->subdev, 1);
	}

	return 0;
}

static const struct dev_pm_ops imx577_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx577_suspend, imx577_resume)
};

static const struct i2c_device_id imx577_id[] = {
	{"imx577", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, imx577_id);

static const struct of_device_id imx577_of_match[] = {
	{ .compatible = "sony,imx577" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx577_of_match);

static struct i2c_driver imx577_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "imx577",
		.pm = &imx577_pm_ops,
		.of_match_table = imx577_of_match,
	},
	.probe  = imx577_probe,
	.remove = imx577_remove,
	.id_table = imx577_id,
};

module_i2c_driver(imx577_i2c_driver);
MODULE_DESCRIPTION("IMX577 MIPI Camera Subdev Driver");
MODULE_LICENSE("GPL");

