#ifndef ADM_MOD_CODEC_H_
#define ADM_MOD_CODEC_H_

struct adm_mod_codec_pdata
{
    bool gpio_master;
    unsigned int adc_pcm_en;
    unsigned int adc_mbo_en;
    unsigned int adc_dsd_en;
    unsigned int adc_pcm_ms;
    unsigned int adc_mdiv;
    unsigned int adc_hpfb;
    unsigned int adc_mode0;
    unsigned int adc_mode1;
    unsigned int adc_rst;
    unsigned int bb_detect;
};

#endif /*ADM_MOD_CODEC_H_*/
