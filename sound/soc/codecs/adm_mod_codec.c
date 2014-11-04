#define DEBUG

#include <linux/module.h>
#include <linux/slab.h>
//#include <linux/pdef/pdef.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_device.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/asoundef.h>
#include <linux/pinctrl/consumer.h>

#include <linux/kthread.h>  // for threads
#include <linux/sched.h>  // for task_struct
#include <linux/time.h>   // for using jiffies  
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/gpio.h>

#include <sound/adm_mod_codec.h>

/*registers*/
#define DAVINCI_MCASP_BASE          0xfa038000

#define DAVINCI_MCASP_PID_REG		0x00
#define DAVINCI_MCASP_PWREMUMGT_REG	0x04

#define DAVINCI_MCASP_PFUNC_REG		0x10
#define DAVINCI_MCASP_PDIR_REG		0x14
#define DAVINCI_MCASP_PDOUT_REG		0x18
#define DAVINCI_MCASP_PDSET_REG		0x1c

#define DAVINCI_MCASP_PDCLR_REG		0x20

#define DAVINCI_MCASP_TLGC_REG		0x30
#define DAVINCI_MCASP_TLMR_REG		0x34

#define DAVINCI_MCASP_GBLCTL_REG	0x44
#define DAVINCI_MCASP_AMUTE_REG		0x48
#define DAVINCI_MCASP_LBCTL_REG		0x4c

#define DAVINCI_MCASP_TXDITCTL_REG	0x50

#define DAVINCI_MCASP_GBLCTLR_REG	0x60
#define DAVINCI_MCASP_RXMASK_REG	0x64
#define DAVINCI_MCASP_RXFMT_REG		0x68
#define DAVINCI_MCASP_RXFMCTL_REG	0x6c

#define DAVINCI_MCASP_ACLKRCTL_REG	0x70
#define DAVINCI_MCASP_AHCLKRCTL_REG	0x74
#define DAVINCI_MCASP_RXTDM_REG		0x78
#define DAVINCI_MCASP_EVTCTLR_REG	0x7c

#define DAVINCI_MCASP_RXSTAT_REG	0x80
#define DAVINCI_MCASP_RXTDMSLOT_REG	0x84
#define DAVINCI_MCASP_RXCLKCHK_REG	0x88
#define DAVINCI_MCASP_REVTCTL_REG	0x8c

#define DAVINCI_MCASP_GBLCTLX_REG	0xa0
#define DAVINCI_MCASP_TXMASK_REG	0xa4
#define DAVINCI_MCASP_TXFMT_REG		0xa8
#define DAVINCI_MCASP_TXFMCTL_REG	0xac

#define DAVINCI_MCASP_ACLKXCTL_REG	0xb0
#define DAVINCI_MCASP_AHCLKXCTL_REG	0xb4
#define DAVINCI_MCASP_TXTDM_REG		0xb8
#define DAVINCI_MCASP_EVTCTLX_REG	0xbc

#define DAVINCI_MCASP_TXSTAT_REG	0xc0
#define DAVINCI_MCASP_TXTDMSLOT_REG	0xc4
#define DAVINCI_MCASP_TXCLKCHK_REG	0xc8
#define DAVINCI_MCASP_XEVTCTL_REG	0xcc

/* Left(even TDM Slot) Channel Status Register File */
#define DAVINCI_MCASP_DITCSRA_REG	0x100
/* Right(odd TDM slot) Channel Status Register File */
#define DAVINCI_MCASP_DITCSRB_REG	0x118
/* Left(even TDM slot) User Data Register File */
#define DAVINCI_MCASP_DITUDRA_REG	0x130
/* Right(odd TDM Slot) User Data Register File */
#define DAVINCI_MCASP_DITUDRB_REG	0x148

/* Serializer n Control Register */
#define DAVINCI_MCASP_XRSRCTL_BASE_REG	0x180
#define DAVINCI_MCASP_XRSRCTL_REG(n)	(DAVINCI_MCASP_XRSRCTL_BASE_REG + \
						(n << 2))

#define DAVINCI_MCASP_RXBUF_REG		0x280

static inline void mcasp_set_bits(void __iomem *reg, u32 val)
{
	__raw_writel(__raw_readl(reg) | val, reg);
}

static inline void mcasp_clr_bits(void __iomem *reg, u32 val)
{
	__raw_writel((__raw_readl(reg) & ~(val)), reg);
}

static inline void mcasp_mod_bits(void __iomem *reg, u32 val, u32 mask)
{
	__raw_writel((__raw_readl(reg) & ~mask) | val, reg);
}

static inline void mcasp_set_reg(void __iomem *reg, u32 val)
{
	__raw_writel(val, reg);
}

static inline u32 mcasp_get_reg(void __iomem *reg)
{
	return (unsigned int)__raw_readl(reg);
}


/* codec private data */
struct adm_mod_private {
	struct snd_soc_codec *codec;
	enum snd_soc_control_type control_type;
	unsigned int sysclk;
	void *control_data;
    struct adm_mod_codec_pdata *gpios;
};
/*
static struct task_struct *thread1;
bool thread_started = false;
int thread_fn(void *data) 
{
    unsigned long j0,j1;
    int delay = 10*HZ;

    //while (time_before(jiffies, j1)) schedule();

    while (true)
    {
        printk("-----------------------\n");
        j0 = jiffies;
        j1 = j0 + delay;

        printk("pfunc: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_PFUNC_REG) );
        printk("pdir: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_PDIR_REG) );
        printk("pdin: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_PDSET_REG) );
        printk("pdout: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_PDOUT_REG) );
        printk("rintctl: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_EVTCTLR_REG) );
        printk("amute: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_AMUTE_REG) );
        printk("rclkchk: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_RXCLKCHK_REG) );
        printk("rfmt: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_RXFMT_REG) );
        printk("aclkxctl: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_ACLKXCTL_REG) );
        printk("aclkrctl: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_ACLKRCTL_REG) );
        printk("ahclkrctl: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_AHCLKRCTL_REG) );
        printk("afsrctl: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_RXFMCTL_REG) );
        printk("revtctl: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_REVTCTL_REG) );
        printk("gblctl: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_GBLCTL_REG) );
        printk("gblctlr: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_GBLCTLR_REG) );
        printk("xrsrctl0: 0x%X\n", (mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_XRSRCTL_REG(0))));
        printk("xrsrctl1: 0x%X\n", (mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_XRSRCTL_REG(1))));
        printk("rbuf0: 0x%X\n", (mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_RXBUF_REG)));
        printk("ditcl: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_TXDITCTL_REG) );
        printk("rtdm: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_RXTDM_REG) );
        printk("rstat: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_RXSTAT_REG) );
        printk("xstat: 0x%X\n", mcasp_get_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_TXSTAT_REG) );

//        mcasp_set_reg(DAVINCI_MCASP_BASE +DAVINCI_MCASP_TXSTAT_REG, 0xFF);
        
        while (time_before(jiffies, j1)) schedule();
    }

    return 0;
}

int thread_init (void) {
    char  our_thread[8]="thread1";
    printk(KERN_INFO "in init");
    thread1 = kthread_create(thread_fn,NULL,our_thread);
    if((thread1))
    {
        printk("in if\n");
        wake_up_process(thread1);
    }

    return 0;
}
*/

static int adm_mod_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;

	dev_dbg(codec->dev, "adm_mod set dai fmt: %i\n", format);
    /*
    dev_dbg(codec->dev, "adm_mod debug pre thread init\n");
    if (thread_started == false)
    {
        dev_dbg(codec->dev, "adm_mod debug thread init\n");
        thread_init(); 
    }
    dev_dbg(codec->dev, "adm_mod debug post thread init\n");
    thread_started = true;
    */

    return 0;
}

static int adm_mod_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *codec_dai)
{
	struct snd_soc_codec *codec = codec_dai->codec;
    struct adm_mod_private *priv = snd_soc_codec_get_drvdata(codec);
	u32 rate = params_rate(params);
	u32 channels = params_channels(params);
	u32 period_sz = params_period_size(params);
	u32 periods = params_periods(params);
	u32 buffer_sz = params_buffer_size(params);
	u32 buffer_bts = params_buffer_bytes(params);

	dev_dbg(codec->dev, "adm_mod hw params rate: %d\n", rate);
	dev_dbg(codec->dev, "adm_mod hw params channels: %d\n", channels);
	dev_dbg(codec->dev, "adm_mod hw params period_sz: %d\n", period_sz);
	dev_dbg(codec->dev, "adm_mod hw params periods: %d\n", periods);
	dev_dbg(codec->dev, "adm_mod hw params buffer_sz: %d\n", buffer_sz);
	dev_dbg(codec->dev, "adm_mod hw params buffer bytes: %d\n", buffer_bts);

    if (priv->gpios->gpio_master)
    {
        //adc reset high
        gpio_set_value(priv->gpios->adc_rst, 1);

        switch (rate)
        {
            case 48000:
                dev_dbg(codec->dev, "adm set rate: mdiv[1] mode0[0] mode1[0]\n");
                gpio_set_value(priv->gpios->adc_mdiv,   1);
                gpio_set_value(priv->gpios->adc_mode0,  0);
                gpio_set_value(priv->gpios->adc_mode1,  0);
                break;
            case 96000:
                dev_dbg(codec->dev, "adm set rate: mdiv[1] mode0[0] mode1[1]\n");
                gpio_set_value(priv->gpios->adc_mdiv,   1);
                gpio_set_value(priv->gpios->adc_mode0,  0);
                gpio_set_value(priv->gpios->adc_mode1,  1);
                break;
            case 192000:
                dev_dbg(codec->dev, "adm set rate: mdiv[1] mode0[1] mode1[0]\n");
                gpio_set_value(priv->gpios->adc_mdiv,   1);
                gpio_set_value(priv->gpios->adc_mode0,  1);
                gpio_set_value(priv->gpios->adc_mode1,  0);
                break;
            case 384000:
                dev_dbg(codec->dev, "adm set rate: mdiv[0] mode0[1] mode1[1]\n");
                gpio_set_value(priv->gpios->adc_mdiv,   0);
                gpio_set_value(priv->gpios->adc_mode0,  1);
                gpio_set_value(priv->gpios->adc_mode1,  1);
                break;
            default:
                return 1;
        }

        //adc reset low
        gpio_set_value(priv->gpios->adc_rst, 0);

        //sleep for 2500fs cycles
        msleep( ( 2500 / (rate / 1000) ) + 1);
    }

    return 0;
}

static int adm_mod_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct adm_mod_private *adm_mod_priv = snd_soc_codec_get_drvdata(codec);

	dev_dbg(codec->dev, "adm_mod set dai sysclk: %i -> dir: %d\n", freq, dir);

	adm_mod_priv->sysclk = freq;
	return 0;
}

static struct snd_soc_dai_ops adm_mod_dai_ops = {
	.hw_params	= adm_mod_hw_params,
	.set_sysclk	= adm_mod_set_dai_sysclk,
	.set_fmt	= adm_mod_set_dai_fmt,
};

static struct snd_soc_dai_driver adm_mod_dai = {
	.name = "adm_mod-hifi",
	.playback = {
		.stream_name = "ADM Playback",
		.channels_min = 2,
		.channels_max = 6,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_384000,
        .formats = ( SNDRV_PCM_FMTBIT_S32_LE ),
    },
	.capture = {
		.stream_name = "ADM Capture",
		.channels_min = 2,
		.channels_max = 6,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_384000,
        .formats = ( SNDRV_PCM_FMTBIT_S32_LE ),
    },
	.ops = &adm_mod_dai_ops,
};

static int adm_mod_probe(struct snd_soc_codec *codec)
{
	struct adm_mod_private *adm_mod = snd_soc_codec_get_drvdata(codec);
	int ret, val;

	codec->control_data = adm_mod->control_data;

    /* Do device init here */

	dev_info(codec->dev, "SPI device initialized\n");
	return 0;
}

static int adm_mod_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_adm_mod = 
{
	.probe = adm_mod_probe,
	.remove = adm_mod_remove,
};

static int adm_mod_request_gpio(unsigned int gpio)
{
    int ret = 0;
    if (!gpio_is_valid(gpio)) {
        printk(KERN_INFO "adm_mod unavailable gpio %d\n", gpio);
        return -1;
    }

    ret = gpio_request(gpio, NULL);
    if (ret < 0)
        return ret;

    //printk(KERN_INFO "setting gpio[%i]: to output\n", gpio);
    gpio_direction_output(gpio, 1);

    return ret;
}

static const struct of_device_id adm_mod_dts_ids[] = {
    { .compatible = "adm_mod-codec" },
    {},
};


static struct adm_mod_codec_pdata *adm_mod_set_pdata_from_of(
						struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
    struct adm_mod_codec_pdata *pdata = NULL;
	const struct of_device_id *match =
			of_match_device(of_match_ptr(adm_mod_dts_ids), &pdev->dev);
    int gpio_master;
	int val, ret = 0;

	if (pdev->dev.platform_data) {
		pdata = pdev->dev.platform_data;
		return pdata;
	} else if (match) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			ret = -ENOMEM;
			goto nodata;
		}
	} else {
		/* control shouldn't reach here. something is wrong */
		ret = -EINVAL;
		goto nodata;
	}

    pdata->gpio_master = false;
	ret = of_property_read_u32(np, "gpio_master", &gpio_master);
    if (gpio_master == 1)
    {
        pdata->gpio_master = true;
        ret = of_property_read_u32(np, "adc_pcm_en", &pdata->adc_pcm_en);
        ret = of_property_read_u32(np, "adc_mbo_en", &pdata->adc_mbo_en);
        ret = of_property_read_u32(np, "adc_dsd_en", &pdata->adc_dsd_en);
        ret = of_property_read_u32(np, "adc_pcm_ms", &pdata->adc_pcm_ms);
        ret = of_property_read_u32(np, "adc_mdiv", &pdata->adc_mdiv);
        ret = of_property_read_u32(np, "adc_hpfb", &pdata->adc_hpfb);
        ret = of_property_read_u32(np, "adc_mode0", &pdata->adc_mode0);
        ret = of_property_read_u32(np, "adc_mode1", &pdata->adc_mode1);
        ret = of_property_read_u32(np, "adc_rst", &pdata->adc_rst);
        ret = of_property_read_u32(np, "bb_detect", &pdata->bb_detect);
    }

	return  pdata;

nodata:
	if (ret < 0) {
		dev_err(&pdev->dev, "Error populating platform data, err %d\n",
			ret);
		pdata = NULL;
	}
	return  pdata;
}


static int adm_mod_pdef_probe(struct platform_device *pdef)
{
	struct adm_mod_private *adm_mod_priv;
	int ret;
	struct pinctrl *pinctrl;

	if (!pdef->dev.platform_data && !pdef->dev.of_node) {
		dev_err(&pdef->dev, "No platform data supplied\n");
		return -EINVAL;
	}

    adm_mod_priv = kzalloc(sizeof(struct adm_mod_private), GFP_KERNEL);
    if (adm_mod_priv == NULL) return -ENOMEM;

	adm_mod_priv->control_data = pdef;
	adm_mod_priv->control_type = SND_SOC_SPI;
	platform_set_drvdata(pdef, adm_mod_priv);

	if ( (ret = snd_soc_register_codec(&pdef->dev,
			&soc_codec_dev_adm_mod, &adm_mod_dai, 1) ) < 0)
    {
        goto soc_register_fail;
    }

	if (! (adm_mod_priv->gpios = adm_mod_set_pdata_from_of(pdef) ) ) {
		dev_err(&pdef->dev, "no platform data\n");
		return -EINVAL;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdef->dev);
	if (IS_ERR(pinctrl))
		dev_warn(&pdef->dev,
				"pins are not configured from the driver\n");

    if (adm_mod_priv->gpios->gpio_master)
    {
        ret = adm_mod_request_gpio(adm_mod_priv->gpios->adc_pcm_en);
        if (ret != 0) goto gpio_fail;
        ret = adm_mod_request_gpio(adm_mod_priv->gpios->adc_mbo_en);
        if (ret != 0) goto gpio_fail;
        ret = adm_mod_request_gpio(adm_mod_priv->gpios->adc_dsd_en);
        if (ret != 0) goto gpio_fail;
        ret = adm_mod_request_gpio(adm_mod_priv->gpios->adc_pcm_ms);
        if (ret != 0) goto gpio_fail;
        ret = adm_mod_request_gpio(adm_mod_priv->gpios->adc_mdiv);
        if (ret != 0) goto gpio_fail;
        ret = adm_mod_request_gpio(adm_mod_priv->gpios->adc_hpfb);
        if (ret != 0) goto gpio_fail;
        ret = adm_mod_request_gpio(adm_mod_priv->gpios->adc_mode0);
        if (ret != 0) goto gpio_fail;
        ret = adm_mod_request_gpio(adm_mod_priv->gpios->adc_mode1);
        if (ret != 0) goto gpio_fail;
        ret = adm_mod_request_gpio(adm_mod_priv->gpios->adc_rst);
        if (ret != 0) goto gpio_fail;
        ret = adm_mod_request_gpio(adm_mod_priv->gpios->bb_detect);
        if (ret != 0) goto gpio_fail;

        gpio_set_value(adm_mod_priv->gpios->adc_pcm_en,      1);
        gpio_set_value(adm_mod_priv->gpios->adc_mbo_en,      0);
        gpio_set_value(adm_mod_priv->gpios->adc_dsd_en,      0);
        gpio_set_value(adm_mod_priv->gpios->adc_pcm_ms,      1);
        gpio_set_value(adm_mod_priv->gpios->adc_mdiv,        1);
        gpio_set_value(adm_mod_priv->gpios->adc_hpfb,        0);
        gpio_set_value(adm_mod_priv->gpios->adc_mode0,       1);
        gpio_set_value(adm_mod_priv->gpios->adc_mode1,       0);
        gpio_set_value(adm_mod_priv->gpios->adc_rst,         0);
        gpio_set_value(adm_mod_priv->gpios->bb_detect,       1); /* let cpld know we are in business */
    }

	return ret;

gpio_fail:
	snd_soc_unregister_codec(&pdef->dev);
soc_register_fail:
	kfree(platform_get_drvdata(pdef));
    return ret;
}

static int adm_mod_pdef_remove(struct platform_device *pdef)
{
	snd_soc_unregister_codec(&pdef->dev);
	kfree(platform_get_drvdata(pdef));
	return 0;
}

static struct platform_driver adm_mod_pdef_driver = 
{
    .driver  = 
    {
        .name   = "adm_mod-codec",
        .owner  = THIS_MODULE,
        .of_match_table = adm_mod_dts_ids,
    },
    .probe  = adm_mod_pdef_probe,
    .remove = adm_mod_pdef_remove,
};

static int __init adm_mod_init(void)
{
    return platform_driver_register(&adm_mod_pdef_driver);
}
module_init(adm_mod_init);

static void __exit adm_mod_exit(void)
{
    platform_driver_unregister(&adm_mod_pdef_driver);
}
module_exit(adm_mod_exit);

MODULE_DESCRIPTION("INCAS3 Amplifier Digitizer Module driver");
MODULE_AUTHOR("Cor Peters");
MODULE_LICENSE("GPL");

