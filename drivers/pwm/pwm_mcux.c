/*
 * Copyright (c) 2019, Linaro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/pwm.h>
#include <soc.h>
#include <fsl_pwm.h>
#include <fsl_clock.h>

#define LOG_LEVEL CONFIG_PWM_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(pwm_mcux);

#define CHANNEL_COUNT 2

struct pwm_mcux_config {
	PWM_Type *base;
	uint8_t index;
	clock_name_t clock_source;
	pwm_clock_prescale_t prescale;
	pwm_mode_t mode;
};

struct pwm_mcux_data {
    // A, B channels have to share the same period!
	uint32_t period_cycles[CHANNEL_COUNT];
	pwm_signal_param_t channel[CHANNEL_COUNT];
};


static status_t PWM_SetupPwmEdgeAlignedCycles(PWM_Type *base,
                      pwm_submodule_t subModule,
                      const pwm_signal_param_t *chnlParams,
					  uint8_t numOfChnls,
                      uint16_t pulseCnt,
                      uint16_t pwmHighPulse)
{
    assert(chnlParams);
    assert(numOfChnls);

    uint8_t i, polarityShift = 0, outputEnableShift = 0;

    if (numOfChnls > 2U)
    {
        /* Each submodule has 2 signals; PWM A & PWM B */
        return kStatus_Fail;
    }

	for (i = 0; i < numOfChnls; i++)
	{
		if (i == 0)
		{
			/* Setup the PWM period */
			base->SM[subModule].INIT = 0;
			/* Indicates the center value */
			base->SM[subModule].VAL0 = (pulseCnt / 2U);
			/* Indicates the end of the PWM period */
			base->SM[subModule].VAL1 = pulseCnt;
		}

		/* Setup the PWM dutycycle */
		if (chnlParams->pwmChannel == kPWM_PwmA)
		{
			base->SM[subModule].VAL2 = 0;
			base->SM[subModule].VAL3 = pwmHighPulse;
		}
		else
		{
			base->SM[subModule].VAL4 = 0;
			base->SM[subModule].VAL5 = pwmHighPulse;
		}

		/* Setup register shift values based on the channel being configured.
		* Also setup the deadtime value
		*/
		if (chnlParams->pwmChannel == kPWM_PwmA)
		{
			polarityShift              = PWM_OCTRL_POLA_SHIFT;
			outputEnableShift          = PWM_OUTEN_PWMA_EN_SHIFT;
			base->SM[subModule].DTCNT0 = PWM_DTCNT0_DTCNT0(chnlParams->deadtimeValue);
		}
		else
		{
			polarityShift              = PWM_OCTRL_POLB_SHIFT;
			outputEnableShift          = PWM_OUTEN_PWMB_EN_SHIFT;
			base->SM[subModule].DTCNT1 = PWM_DTCNT1_DTCNT1(chnlParams->deadtimeValue);
		}

		/* Setup signal active level */
		if ((bool)chnlParams->level == kPWM_HighTrue)
		{
			base->SM[subModule].OCTRL &= ~((uint16_t)1U << (uint16_t)polarityShift);
		}
		else
		{
			base->SM[subModule].OCTRL |= ((uint16_t)1U << (uint16_t)polarityShift);
		}

		/* Enable PWM output */
		base->OUTEN |= ((uint16_t)1U << ((uint16_t)outputEnableShift + (uint16_t)subModule));

		chnlParams++;
	}

    return kStatus_Success;
}

static inline void PWM_UpdatePwmEdgeAlignedPulseCycles(PWM_Type *base,
                            pwm_submodule_t subModule,
                            pwm_channels_t pwmSignal,
                            uint16_t pwmHighPulse)
{
    if (pwmSignal == kPWM_PwmA)
    {
        base->SM[subModule].VAL2 = 0;
        base->SM[subModule].VAL3 = pwmHighPulse;
    }
    else
    {
        base->SM[subModule].VAL4 = 0;
        base->SM[subModule].VAL5 = pwmHighPulse;
    }
}

static int mcux_pwm_pin_set(struct device *dev, u32_t ch,
			    u32_t period_cycles, u32_t pulse_cycles,
			    pwm_flags_t flags)
{
	const struct pwm_mcux_config *config = dev->config->config_info;
	struct pwm_mcux_data *data = dev->driver_data;

	if (ch >= CHANNEL_COUNT) {
		LOG_ERR("Invalid channel");
		return -EINVAL;
	}

	if (flags) {
		/* PWM polarity not supported (yet?) */
		return -ENOTSUP;
	}

    if (period_cycles > UINT16_MAX) {
		/* 16-bit resolution */
		LOG_ERR("Too long period (%u), adjust ch prescaler!",
			period_cycles);
		/* TODO: dynamically adjust prescaler */
		return -EINVAL;
	}

	if (pulse_cycles > period_cycles) {
		pulse_cycles = period_cycles;
	}

	if (period_cycles != data->period_cycles[ch]) {
        // Update frequency and dutycycle
        status_t status;

		data->period_cycles[ch] = period_cycles;

		if (period_cycles == 0) {
			PWM_StopTimer(config->base, 1U << config->index);
			return 0;
		}

		// StopTimer would produce plosive when changing frequency of a buzzer!
		 //PWM_StopTimer(config->base, 1U << config->index);
		PWM_SetPwmLdok(config->base, 1U << config->index, false);

		status = PWM_SetupPwmEdgeAlignedCycles(config->base, config->index, &data->channel[0], 2,
                                              (uint16_t)period_cycles, (uint16_t)pulse_cycles);
		if (status != kStatus_Success) {
			LOG_ERR("Could not set up pwm");
			return -ENOTSUP;
		}
		PWM_SetPwmLdok(config->base, 1U << config->index, true);
		PWM_StartTimer(config->base, 1U << config->index);
	} else {
		// Update dutycycle only
		PWM_SetPwmLdok(config->base, 1U << config->index, false);
		PWM_UpdatePwmEdgeAlignedPulseCycles(config->base, config->index,
				(ch == 0) ? kPWM_PwmA : kPWM_PwmB, (u16_t)pulse_cycles);
		PWM_SetPwmLdok(config->base, 1U << config->index, true);
		PWM_StartTimer(config->base, 1U << config->index);
	}

	return 0;
}


static int mcux_pwm_get_cycles_per_sec(struct device *dev, u32_t pwm,
				       u64_t *cycles)
{
	const struct pwm_mcux_config *config = dev->config->config_info;

	*cycles = CLOCK_GetFreq(config->clock_source) >> config->prescale;

	return 0;
}

static int pwm_mcux_init(struct device *dev)
{
	const struct pwm_mcux_config *config = dev->config->config_info;
	struct pwm_mcux_data *data = dev->driver_data;
	pwm_config_t pwm_config;
	status_t status;

	PWM_GetDefaultConfig(&pwm_config);
	pwm_config.prescale = config->prescale;
	pwm_config.reloadLogic = kPWM_ReloadPwmFullCycle;

	status = PWM_Init(config->base, config->index, &pwm_config);
	if (status != kStatus_Success) {
		LOG_ERR("Unable to init PWM");
		return -EIO;
	}

	/* Disable fault sources */
	((PWM_Type *)config->base)->SM[config->index].DISMAP[0] = 0x0000;
	((PWM_Type *)config->base)->SM[config->index].DISMAP[1] = 0x0000;

	data->channel[0].pwmChannel = kPWM_PwmA;
	data->channel[0].level = kPWM_HighTrue;
	data->channel[1].pwmChannel = kPWM_PwmB;
	data->channel[1].level = kPWM_HighTrue;

	return 0;
}

static const struct pwm_driver_api pwm_mcux_driver_api = {
	.pin_set = mcux_pwm_pin_set,
	.get_cycles_per_sec = mcux_pwm_get_cycles_per_sec,
};

#define PWM_DEVICE_INIT_MCUX(n)			  \
	static struct pwm_mcux_data pwm_mcux_data_ ## n;		  \
									  \
	static const struct pwm_mcux_config pwm_mcux_config_ ## n = {     \
		.base = (void *)DT_PWM_MCUX_ ## n ## _BASE_ADDRESS,	  \
		.index = DT_PWM_MCUX_ ## n ## _INDEX,			  \
		.mode = kPWM_EdgeAligned,				  \
		.prescale = kPWM_Prescale_Divide_64,			  \
		.clock_source = kCLOCK_IpgClk,				  \
	};								  \
									  \
	DEVICE_AND_API_INIT(pwm_mcux_ ## n,				  \
			    DT_PWM_MCUX_ ## n ## _NAME,			  \
			    pwm_mcux_init,				  \
			    &pwm_mcux_data_ ## n,			  \
			    &pwm_mcux_config_ ## n,			  \
			    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,\
			    &pwm_mcux_driver_api);

#ifdef CONFIG_FLEXPWM1_PWM0
PWM_DEVICE_INIT_MCUX(0)
#endif

#ifdef CONFIG_FLEXPWM1_PWM1
PWM_DEVICE_INIT_MCUX(1)
#endif

#ifdef CONFIG_FLEXPWM1_PWM2
PWM_DEVICE_INIT_MCUX(2)
#endif

#ifdef CONFIG_FLEXPWM1_PWM3
PWM_DEVICE_INIT_MCUX(3)
#endif

#ifdef CONFIG_FLEXPWM2_PWM0
PWM_DEVICE_INIT_MCUX(4)
#endif

#ifdef CONFIG_FLEXPWM2_PWM1
PWM_DEVICE_INIT_MCUX(5)
#endif

#ifdef CONFIG_FLEXPWM2_PWM2
PWM_DEVICE_INIT_MCUX(6)
#endif

#ifdef CONFIG_FLEXPWM2_PWM3
PWM_DEVICE_INIT_MCUX(7)
#endif

#ifdef CONFIG_FLEXPWM3_PWM0
PWM_DEVICE_INIT_MCUX(8)
#endif

#ifdef CONFIG_FLEXPWM3_PWM1
PWM_DEVICE_INIT_MCUX(9)
#endif

#ifdef CONFIG_FLEXPWM3_PWM2
PWM_DEVICE_INIT_MCUX(10)
#endif

#ifdef CONFIG_FLEXPWM3_PWM3
PWM_DEVICE_INIT_MCUX(11)
#endif

#ifdef CONFIG_FLEXPWM4_PWM0
PWM_DEVICE_INIT_MCUX(12)
#endif

#ifdef CONFIG_FLEXPWM4_PWM1
PWM_DEVICE_INIT_MCUX(13)
#endif

#ifdef CONFIG_FLEXPWM4_PWM2
PWM_DEVICE_INIT_MCUX(14)
#endif

#ifdef CONFIG_FLEXPWM4_PWM3
PWM_DEVICE_INIT_MCUX(15)
#endif
