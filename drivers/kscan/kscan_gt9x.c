/*
 * Copyright (c) 2020 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/kscan.h>
#include <drivers/i2c.h>
#include <drivers/gpio.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(gt9x, CONFIG_KSCAN_LOG_LEVEL);

#define TOUCH_CONTINUE

#define GT9X_I2C_ADDRESS_A	0x14
#define GT9X_I2C_ADDRESS_B	0x5D

#define GT9X_COOR_ADDR    0x814E
#define GT9X_REG_SLEEP         0x8040
#define GT9X_REG_SENSOR_ID     0x814A
#define GT9X_REG_CONFIG_DATA   0x8050
#define GT9X_REG_VERSION       0x8140

#define GT9X_REG_NUMBER	256
#define GT9X_REG_ADDR_LEN	2

#define GT9X_VERSION_LEN		6

#define GT9X_MAX_TOUCH	5
#define	GT9X_PER_TOUCH_LEN	8
#define GT9X_TOUCH_STATUS_LEN	1
#define GT9X_TOUCH_RESERVED_LEN	1

static uint8_t gt9x_cfg_regs[] ={
  0x97,0x20,0x03,0xE0,0x01,0x0A,0x35,0x04,0x00,0x69,
  0x09,0x0F,0x50,0x32,0x33,0x11,0x00,0x32,0x11,0x11,
  0x28,0x8C,0xAA,0xDC,0x58,0x04,0x00,0x00,0x1E,0x3C,
  0x00,0x00,0x00,0x31,0x00,0x00,0x00,0x00,0x00,0x40,
  0x32,0x00,0x00,0x50,0x38,0x00,0x8D,0x20,0x16,0x4E,
  0x4C,0x7C,0x05,0x28,0x3E,0x28,0x0D,0x43,0x24,0x00,
  0x01,0x39,0x6B,0xC0,0x94,0x84,0x2D,0x00,0x54,0xB0,
  0x41,0x9D,0x49,0x8D,0x52,0x7F,0x5A,0x75,0x62,0x6C,
  0x42,0x50,0x14,0x00,0x00,0x00,0x00,0xF0,0x50,0x3C,
  0x88,0x88,0x27,0x50,0x3C,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x00,0x02,0x78,
  0x0A,0x50,0xFF,0xE4,0x04,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x3C,0xB0,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x56,0xA2,0x07,0x50,0x1E,
  0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,
  0x0F,0x10,0x12,0x15,0x16,0x17,0x18,0x19,0x1A,0x1B,
  0x1D,0x1F,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0x1F,0x1E,0x1D,0x1C,0x1B,0x1A,0x19,0x18,
  0x17,0x15,0x14,0x13,0x12,0xFF,0xFF,0xFF,0xFF,0xFF,
  0xFF,0xFF,0xFF,0xFF,0x00,0x30,0x7F,0x7F,0x7F,0xFF,
  0x54,0x64,0x00,0x80,0x46,0x07,0x50,0x3C,0x32,0x14,
  0x0A,0x64,0x32,0x00,0x00,0x00,0x00,0x11,0x02,0x62,
  0x32,0x03,0x14,0x50,0x0C,0xE2,0x14,0x50,0x00,0x54,
  0x10,0x00,0x32,0xA2,0x07,0x64,0xA4,0xB6,0x01
};

struct gt9x_config {
	char *i2c_name;
	u8_t i2c_address;
	char *int_port ;
	u32_t int_pin;
	u32_t int_flags;
	char *rst_port;
	u32_t rst_pin;
	u32_t rst_flags;
};

struct gt9x_data {
	struct device *i2c;
	struct device *gpio_int;
	struct device *gpio_rst;
	kscan_callback_t callback;
	struct k_work work;
#ifdef CONFIG_KSCAN_GT9X_INTERRUPT	
	struct gpio_callback int_gpio_cb;
#else
	struct k_timer timer;
#endif
	struct device *dev;
	
};

struct touch_point_info{
	u8_t id;
	u16_t x;
	u16_t y;
	u16_t size;
};


static int gt9x_write_regs(struct device *dev, u8_t dev_addr,
	u16_t reg_addr,	u8_t *regs, u16_t reg_num)
{
	u8_t buf[GT9X_REG_ADDR_LEN + GT9X_REG_NUMBER] =
		{reg_addr >> 8, reg_addr & 0xFF,};

	if(reg_num > GT9X_REG_NUMBER){
		return -1;
	}
	memcpy(&buf[GT9X_REG_ADDR_LEN], regs, reg_num);

	return i2c_write(dev, buf, reg_num+GT9X_REG_ADDR_LEN, dev_addr);
}


static int gt9x_read_regs(struct device *dev, u8_t dev_addr,
	u16_t reg_addr,	u8_t *regs, u16_t reg_num)
{
	u8_t addr[2] = {reg_addr >> 8, reg_addr & 0xFF};
	//LOG_DBG("read address %x %x", addr[0], addr[1]);
	i2c_write(dev,addr,2,dev_addr);
	//LOG_DBG("start read");
	//return i2c_read(dev, regs, reg_num, dev_addr);
	return i2c_write_read(dev, dev_addr, addr, 2, regs, reg_num);
}

#ifdef CONFIG_KSCAN_GT9X_INTERRUPT
static void gt9x_isr_handler(struct device *dev,
				  struct gpio_callback *cb, u32_t pins)
{
	printk("touch int\n");
	const struct gt9x_config *config = dev->config->config_info;
	struct gt9x_data *drv_data =
		CONTAINER_OF(cb, struct gt9x_data, int_gpio_cb);

	gpio_pin_interrupt_configure(dev, config->int_pin,
		GPIO_INT_DISABLE);

	k_work_submit(&drv_data->work);
}
#endif

static void gt9x_get_version(struct device *dev)
{
	const struct gt9x_config *config = dev->config->config_info;
	struct gt9x_data *data = dev->driver_data;
	u8_t version[GT9X_VERSION_LEN];
	int ret = 0;
	
	ret = gt9x_read_regs(data->i2c,config->i2c_address,
	  GT9X_REG_VERSION, version, GT9X_VERSION_LEN);

	if (ret != 0) {
	  LOG_ERR("TOUCH I2C read reg fail");
	  return -EINVAL;
	}

	version[5] = 0;
	printk("Touch device %s\n", version);
}

static int gt9x_read(struct device *dev)
{
	const struct gt9x_config *config = dev->config->config_info;
	struct gt9x_data *data = dev->driver_data;
	u8_t buf[GT9X_PER_TOUCH_LEN*GT9X_MAX_TOUCH];
	int ret = 0;
	u16_t status = 0;
	u8_t i,j;
	u8_t touch_number = 0;
	struct touch_point_info cur_touch[GT9X_MAX_TOUCH] = {0};

	static u8_t pre_touch_number = 0;
	static struct touch_point_info pre_touch[GT9X_MAX_TOUCH] = {0};
	
	//read touch status
	ret = gt9x_read_regs(data->i2c, config->i2c_address,
		GT9X_COOR_ADDR, &status, 1);
	if (ret) {
		LOG_ERR("Could not read point");
		return -EIO;
	}

	//LOG_DBG("read %x", status);

	/* No data */
	if(status == 0){
		return 0;
	}

	do{
		//data no ready
		if((status & 0x80) == 0){
		  break;
		}

		touch_number = status & 0x0f;
		if(touch_number > GT9X_MAX_TOUCH){
			break;
		}

		if(touch_number == 0){
			for(i=0; i<pre_touch_number; i++){
				LOG_INF("Release[%d] %u %u", pre_touch[i].id, pre_touch[i].x, pre_touch[i].y);
				//printk("Release %u %u\n", pre_touch[i].x, pre_touch[i].y);
				if(data->callback != NULL){
					data->callback(dev, pre_touch[i].y, pre_touch[i].x, false);
				}
			}
			pre_touch_number = 0;
			break;
		}

		//LOG_DBG("Touch %d", touch_number);

		ret = gt9x_read_regs(data->i2c, config->i2c_address,
			GT9X_COOR_ADDR+GT9X_TOUCH_STATUS_LEN,
			buf, touch_number*GT9X_PER_TOUCH_LEN);

		if (ret) {
			LOG_ERR("Could not read point");
			ret = -EIO;
			break;
		}

		//check press key
		for(i=0; i<touch_number; i++){
			cur_touch[i].id = buf[i*GT9X_PER_TOUCH_LEN];
			cur_touch[i].x = buf[i*GT9X_PER_TOUCH_LEN+2];
			cur_touch[i].x = (cur_touch[i].x<<8) | buf[i*GT9X_PER_TOUCH_LEN+1];
			cur_touch[i].y = buf[i*GT9X_PER_TOUCH_LEN+4];
			cur_touch[i].y = (cur_touch[i].y<<8) | buf[i*GT9X_PER_TOUCH_LEN+3];
			cur_touch[i].size = buf[i*GT9X_PER_TOUCH_LEN+6];
			cur_touch[i].size = (cur_touch[i].size<<8) | buf[i*GT9X_PER_TOUCH_LEN+5];
			//LOG_INF("Current Touch %u %u", cur_touch[i].x, cur_touch[i].y);
#ifdef TOUCH_CONTINUE
			LOG_INF("Touch[%d] %u %u", cur_touch[i].id, cur_touch[i].x, cur_touch[i].y);
			//printk("Touch %u %u -- %x\n", cur_touch[i].x, cur_touch[i].y, data->callback);
			if(data->callback != NULL){
				data->callback(dev, cur_touch[i].y, cur_touch[i].x, true);
			}
#else
			for(j=0; j<pre_touch_number; j++){
				if(cur_touch[i].id == pre_touch[j].id){
					break;
				}
			}

			if(j>=pre_touch_number){
				LOG_INF("Touch[%d] %u %u", cur_touch[i].id, cur_touch[i].x, cur_touch[i].y);
				//printk("Touch %u %u -- %x\n", cur_touch[i].x, cur_touch[i].y, data->callback);
				if(data->callback != NULL){
					data->callback(dev, cur_touch[i].y, cur_touch[i].x, true);
				}
			}
#endif
		}

		//check release key
		for(i=0; i<pre_touch_number; i++){
			for(j=0; j<touch_number; j++){
				if(cur_touch[j].id == pre_touch[i].id){
					break;
				}
			}

			if(j>=touch_number){
				LOG_INF("Release[%d] %u %u", pre_touch[i].id, pre_touch[i].x, pre_touch[i].y);
				//printk("Release %u %u\n", cur_touch[i].x, cur_touch[i].y);
				if(data->callback != NULL){
					data->callback(dev, pre_touch[i].y, pre_touch[i].x, false);
				}
			}
		}

		if(touch_number != pre_touch_number){
			LOG_DBG("====================================================================");
			LOG_DBG("touch %d pre touch %d", touch_number, pre_touch_number);
			for(i=0; i<touch_number; i++){
				LOG_DBG("	touch[%d] %u %u", cur_touch[i].id, cur_touch[i].x, cur_touch[i].y);
			}
			for(i=0; i<pre_touch_number; i++){
				LOG_DBG("	pretouch[%d] %u %u", pre_touch[i].id, pre_touch[i].x, pre_touch[i].y);
			}
			LOG_DBG("====================================================================\n");
		}

		memcpy(pre_touch, cur_touch, sizeof(cur_touch));
		pre_touch_number = touch_number;
	}while(0);

	status = 0;
	gt9x_write_regs(data->i2c, config->i2c_address,
		GT9X_COOR_ADDR, &status, GT9X_TOUCH_STATUS_LEN);

#ifdef CONFIG_KSCAN_GT9X_INTERRUPT
	gpio_pin_interrupt_configure(data->gpio_int,
		   config->int_pin, GPIO_INT_EDGE_TO_ACTIVE);
#endif


	return ret;
}

#ifndef CONFIG_KSCAN_GT9X_INTERRUPT
static void gt9x_timer_handler(struct k_timer *timer)
{
	struct gt9x_data *data =
		CONTAINER_OF(timer, struct gt9x_data, timer);

	k_work_submit(&data->work);
}
#endif

static void gt9x_work_handler(struct k_work *work)
{
	struct gt9x_data *data =
		CONTAINER_OF(work, struct gt9x_data, work);

	gt9x_read(data->dev);
	//gt9x_get_version(data->dev);
}

static int gt9x_configure(struct device *dev, kscan_callback_t callback)
{
	struct gt9x_data *data = dev->driver_data;

	if (!callback) {
		return -EINVAL;
	}

	data->callback = callback;

	return 0;
}

static int gt9x_enable_callback(struct device *dev)
{
	struct gt9x_data *data = dev->driver_data;
	
#ifdef CONFIG_KSCAN_GT9X_INTERRUPT
	gpio_add_callback(data->gpio_int, &data->int_gpio_cb);
#else
	k_timer_start(&data->timer, K_MSEC(CONFIG_KSCAN_GT9X_PERIOD),
		      K_MSEC(CONFIG_KSCAN_GT9X_PERIOD));
#endif
	return 0;
}

static int gt9x_disable_callback(struct device *dev)
{
	struct gt9x_data *data = dev->driver_data;
#ifdef CONFIG_KSCAN_GT9X_INTERRUPT
	gpio_remove_callback(data->gpio_int, &data->int_gpio_cb);
#else
	k_timer_stop(&data->timer);
#endif
	return 0;
}

static int gt9x_init(struct device *dev)
{
	const struct gt9x_config *config = dev->config->config_info;
	struct gt9x_data *data = dev->driver_data;
	u8_t version[GT9X_VERSION_LEN];
	int ret = -1;
	u16_t check_sum;
	u16_t reg_num;

#ifdef DT_INST_0_COODIX_GT9X_INT_GPIOS_CONTROLLER
	data->gpio_int = device_get_binding(config->int_port);
	if (data->gpio_int == NULL) {
		LOG_ERR("Could not find INT GPIO device");
		return -EINVAL;
	}
#endif

#ifdef DT_INST_0_COODIX_GT9X_RESET_GPIOS_CONTROLLER
	data->gpio_rst = device_get_binding(config->rst_port);
	if (data->gpio_rst == NULL) {
		LOG_ERR("Could not find rst GPIO device");
		return -EINVAL;
	}
#endif

	/* configure i2c address*/
	if (data->gpio_int != NULL && data->gpio_rst != NULL) {
		if (gpio_pin_configure(data->gpio_int, config->int_pin,
			       GPIO_OUTPUT | config->int_flags)) {
            LOG_ERR("Unable to configure int GPIO pin %u", config->int_pin);
            return -EINVAL;
	    }

		if (gpio_pin_configure(data->gpio_rst, config->rst_pin,
			       GPIO_OUTPUT | config->rst_flags)) {
            LOG_ERR("Unable to configure rst GPIO pin %u", config->rst_pin);
            return -EINVAL;
	    }

        gpio_pin_set(data->gpio_rst, config->rst_pin, 0);
		gpio_pin_set(data->gpio_int, config->int_pin, 0);
		k_sleep(K_MSEC(1));

		if(config->i2c_address == GT9X_I2C_ADDRESS_A) {
            gpio_pin_set(data->gpio_int, config->int_pin, 1);
		} else if (config->i2c_address == GT9X_I2C_ADDRESS_B) {
            gpio_pin_set(data->gpio_int, config->int_pin, 0);
		} else {
			LOG_ERR("No support I2C address %02x", config->i2c_address);
			return -EINVAL;
		}

        k_sleep(K_MSEC(1));
        gpio_pin_set(data->gpio_rst, config->rst_pin, 1);

        k_sleep(K_MSEC(6));
	}


    if(data->gpio_int != NULL){
        gpio_pin_configure(data->gpio_int, config->int_pin, GPIO_INPUT | config->int_flags);
#ifdef CONFIG_KSCAN_GT9X_INTERRUPT

		ret = gpio_pin_interrupt_configure(data->gpio_int,
		   config->int_pin, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret != 0) {
			LOG_ERR("Error %d: failed to configure pin interrupt %d '%s'\n",
				ret, config->int_pin, config->int_port);
			return -EINVAL;
		}
        gpio_init_callback(&data->int_gpio_cb, gt9x_isr_handler, BIT(config->int_pin));
#endif
    }

	data->i2c = device_get_binding(config->i2c_name);
	if (data->i2c == NULL) {
		LOG_ERR("Could not find I2C device");
		return -EINVAL;
	}

	ret = gt9x_read_regs(data->i2c,config->i2c_address,
		GT9X_REG_VERSION, version, GT9X_VERSION_LEN);

	if (ret != 0) {
		LOG_ERR("I2C read reg fail");
		return -EINVAL;
	}

	if(version[0] != '9' || version[1] != '1' || version[2] != '7' || version[3] != 'S'){
		LOG_ERR("GT917S Version no match");
		return -EINVAL;
	}


	reg_num = sizeof(gt9x_cfg_regs);
	check_sum = 0;
	for (int i = 0; i < reg_num; i+=2)
	{
		check_sum += (gt9x_cfg_regs[i]<<8) + gt9x_cfg_regs[i+1];
	}

	u8_t buf[reg_num+3];
	memcpy(buf, gt9x_cfg_regs, reg_num);
	buf[reg_num] = (check_sum >> 8) & 0xFF;
	buf[reg_num+1] =  check_sum & 0xFF;
	reg_num += 2;

	ret = gt9x_write_regs(data->i2c,config->i2c_address,
		GT9X_REG_CONFIG_DATA, buf, reg_num);

	if (ret != 0) {
		LOG_ERR("confiure gt9x fail\n");
		return -EINVAL;
	}

	k_sleep(K_MSEC(50));
/*
	memset(buf, 0, sizeof(buf));
	gt9x_read_regs(data->i2c,config->i2c_address,
		GT9X_REG_CONFIG_DATA, buf, sizeof(gt9x_cfg_regs));

	ret = memcmp(buf, gt9x_cfg_regs, sizeof(gt9x_cfg_regs));
	LOG_ERR("cmp cfg result %d\n", ret);
*/



	data->dev = dev;


	k_work_init(&data->work, gt9x_work_handler);
#ifndef CONFIG_KSCAN_GT9X_INTERRUPT
	k_timer_init(&data->timer, gt9x_timer_handler, NULL);
#endif

	gt9x_enable_callback(dev);

	return 0;
}


static const struct kscan_driver_api gt9x_driver_api = {
	.config = gt9x_configure,
	.enable_callback = gt9x_enable_callback,
	.disable_callback = gt9x_disable_callback,
};

static const struct gt9x_config gt9x_config = {
	.i2c_name = DT_INST_0_COODIX_GT9X_BUS_NAME,
	.i2c_address = DT_INST_0_COODIX_GT9X_BASE_ADDRESS,
	.int_port = DT_INST_0_COODIX_GT9X_INT_GPIOS_CONTROLLER,
	.int_pin = DT_INST_0_COODIX_GT9X_INT_GPIOS_PIN,
	.int_flags = DT_INST_0_COODIX_GT9X_INT_GPIOS_FLAGS,
	.rst_port = DT_INST_0_COODIX_GT9X_RESET_GPIOS_CONTROLLER,
	.rst_pin = DT_INST_0_COODIX_GT9X_RESET_GPIOS_PIN,
	.rst_flags = DT_INST_0_COODIX_GT9X_RESET_GPIOS_FLAGS,
};

static struct gt9x_data gt9x_data;

DEVICE_AND_API_INIT(GT9X, DT_INST_0_COODIX_GT9X_LABEL, gt9x_init,
		    &gt9x_data, &gt9x_config,
		    POST_KERNEL, CONFIG_KSCAN_INIT_PRIORITY,
		    &gt9x_driver_api);
