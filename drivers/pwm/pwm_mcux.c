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
	uint32_t period_cycles;
	pwm_signal_param_t channel;
};


static void PWM_UpdatePwmDutycycleFloat(PWM_Type *base,
                            pwm_submodule_t subModule,
                            pwm_channels_t pwmSignal,
                            pwm_mode_t currPwmMode,
                            float dutyCyclePercent)
{
    assert(((uint8_t)dutyCyclePercent) <= 100);
    assert(pwmSignal < 2);
    uint16_t pulseCnt = 0, pwmHighPulse = 0;
    int16_t modulo = 0;

    switch (currPwmMode)
    {
        case kPWM_SignedCenterAligned:
            modulo   = base->SM[subModule].VAL1;
            pulseCnt = modulo * 2;
            /* Calculate pulse width */
            pwmHighPulse = (uint16_t)(((float)pulseCnt * dutyCyclePercent) / 100.0);

            /* Setup the PWM dutycycle */
            if (pwmSignal == kPWM_PwmA)
            {
                base->SM[subModule].VAL2 = (-(pwmHighPulse / 2));
                base->SM[subModule].VAL3 = (pwmHighPulse / 2);
            }
            else
            {
                base->SM[subModule].VAL4 = (-(pwmHighPulse / 2));
                base->SM[subModule].VAL5 = (pwmHighPulse / 2);
            }
            break;
        case kPWM_CenterAligned:
            pulseCnt = base->SM[subModule].VAL1;
            /* Calculate pulse width */
            pwmHighPulse = (uint16_t)(((float)pulseCnt * dutyCyclePercent) / 100.0);

            /* Setup the PWM dutycycle */
            if (pwmSignal == kPWM_PwmA)
            {
                base->SM[subModule].VAL2 = ((pulseCnt - pwmHighPulse) / 2);
                base->SM[subModule].VAL3 = ((pulseCnt + pwmHighPulse) / 2);
            }
            else
            {
                base->SM[subModule].VAL4 = ((pulseCnt - pwmHighPulse) / 2);
                base->SM[subModule].VAL5 = ((pulseCnt + pwmHighPulse) / 2);
            }
            break;
        case kPWM_SignedEdgeAligned:
            modulo   = base->SM[subModule].VAL1;
            pulseCnt = modulo * 2;
            /* Calculate pulse width */
            pwmHighPulse = (uint16_t)(((float)pulseCnt * dutyCyclePercent) / 100.0);

            /* Setup the PWM dutycycle */
            if (pwmSignal == kPWM_PwmA)
            {
                base->SM[subModule].VAL2 = (-modulo);
                base->SM[subModule].VAL3 = (-modulo + pwmHighPulse);
            }
            else
            {
                base->SM[subModule].VAL4 = (-modulo);
                base->SM[subModule].VAL5 = (-modulo + pwmHighPulse);
            }
            break;
        case kPWM_EdgeAligned:
            pulseCnt = base->SM[subModule].VAL1;
            /* Calculate pulse width */
            pwmHighPulse = (uint16_t)(((float)pulseCnt * dutyCyclePercent) / 100.0);

            /* Setup the PWM dutycycle */
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
            break;
        default:
            break;
    }
}

static int mcux_pwm_pin_set(struct device *dev, u32_t pwm,
			    u32_t period_cycles, u32_t pulse_cycles,
			    pwm_flags_t flags)
{
	const struct pwm_mcux_config *config = dev->config->config_info;
	struct pwm_mcux_data *data = dev->driver_data;
	u8_t duty_cycle;
	float duty_cycle_float;

	if (pwm >= CHANNEL_COUNT + 2) {
		LOG_ERR("Invalid channel");
		return -EINVAL;
	}

	if (flags) {
		/* PWM polarity not supported (yet?) */
		return -ENOTSUP;
	}

	if ((period_cycles == 0) || (pulse_cycles > period_cycles)) {
		LOG_ERR("Invalid combination: period_cycles=%u, "
			"pulse_cycles=%u", period_cycles, pulse_cycles);
		return -EINVAL;
	}

	if (period_cycles > UINT16_MAX) {
		/* 16-bit resolution */
		LOG_ERR("Too long period (%u), adjust pwm prescaler!",
			period_cycles);
		/* TODO: dynamically adjust prescaler */
		return -EINVAL;
	}

	duty_cycle = 100 * pulse_cycles / period_cycles;
	duty_cycle_float = (float)(100 * pulse_cycles) / (float)period_cycles;

	/* FIXME: Force re-setup even for duty-cycle update */
	//if (period_cycles != data->period_cycles) {
    // Update dutycycle and frequency
	if (pwm >=  CHANNEL_COUNT) {
		uint32_t clock_freq;
		uint32_t pwm_freq;
		status_t status;

		pwm -= 2;

		data->period_cycles = period_cycles;

		LOG_DBG("SETUP dutycycle to %u\n", duty_cycle);

		clock_freq = CLOCK_GetFreq(config->clock_source);
		pwm_freq = (clock_freq >> config->prescale) / period_cycles;

		if (pwm_freq == 0) {
			LOG_ERR("Could not set up pwm_freq=%d", pwm_freq);
			return -EINVAL;
		}

		PWM_StopTimer(config->base, 1U << config->index);

		data->channel.pwmChannel = (pwm == 0) ? kPWM_PwmA : kPWM_PwmB;
		data->channel.dutyCyclePercent = duty_cycle;

		status = PWM_SetupPwm(config->base, config->index,
				      &data->channel, 1,
				      config->mode, pwm_freq, clock_freq);
		if (status != kStatus_Success) {
			LOG_ERR("Could not set up pwm");
			return -ENOTSUP;
		}

		PWM_UpdatePwmDutycycleFloat(config->base, config->index,
				       (pwm == 0) ? kPWM_PwmA : kPWM_PwmB,
				       config->mode, duty_cycle_float);

		PWM_SetPwmLdok(config->base, 1U << config->index, true);

		PWM_StartTimer(config->base, 1U << config->index);
	} else {
		// Update dutycycle only
		//PWM_UpdatePwmDutycycle(config->base, config->index,
		//		       (pwm == 0) ? kPWM_PwmA : kPWM_PwmB,
		//		       config->mode, duty_cycle);
		PWM_UpdatePwmDutycycleFloat(config->base, config->index,
				       (pwm == 0) ? kPWM_PwmA : kPWM_PwmB,
				       config->mode, duty_cycle_float);
		PWM_SetPwmLdok(config->base, 1U << config->index, true);
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

	//data->channel[0].pwmChannel = kPWM_PwmA;
	//data->channel[0].level = kPWM_HighTrue;
	//data->channel[1].pwmChannel = kPWM_PwmB;
	data->channel.level = kPWM_HighTrue;

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
