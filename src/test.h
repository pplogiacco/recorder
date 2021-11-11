
#ifndef XTEST_H
#define	XTEST_H

//
//#define __VAMP1K_TEST
//#define __NOFLASH    // Use RAM to store config
//#define __NOUSB      // Force to use RF ( USB not connect )
//
#ifdef __VAMP1K_TEST
#undef __VAMP1K
#else
#define __VAMP1K
#endif

// Test HW
//#define __VAMP1K_TEST_HW                    
//#define __VAMP1K_TEST_RESET
//#define __VAMP1K_TEST_TIMERS
//#define __VAMP1K_TEST_CONFIG 
//#define __VAMP1K_TEST_ADG
//#define __VAMP1K_TEST_USB
//#define __VAMP1K_TEST_RTCC
//#define __VAMP1K_TEST_SLEEP
//#define __VAMP1K_TEST_BATTERY
//#define __VAMP1K_TEST_BATLEV
//#define __VAMP1K_TEST_SST26
//#define __VAMP1K_TEST_DDE


#define __VAMP1K_TEST_measurement_save
//#define __VAMP1K_TEST_measurement_delete 
//#define __VAMP1K_TEST_adc_printf           // Test Acquire/ADC HW
//#define __VAMP1K_TEST_measurement_printf   // Test Measurement format 
//#define __VAMP1K_TEST_measurement_DATAVIS  // send ADC to serial mc datavis
//#define __VAMP1K_TEST_measurement_save

#define FORCED_TYPESET _AV05
//    _SIG0 = 0x02, // (02) Test signal  { <sig_fq>, <sig_maxa>, <adc_fq>, <res_scale>, [<dT>,<a>],[...] }           
//    _AV00 = 0x0A, // (10) Aeolian Vibration, RAW { <ET>,<WS>,<adc_fq> <res_scale>,[<dT>,<s>],...}
//    _AV01 = 0x0D, // (13) Aeolian Vibration, P-P { <ET>,<WS>,<adc_fq> <res_scale,[<dT>,<sp>],...}
//    _AV02 = 0x0C, // (12) Aeolian Vibration, FFT Real { <ET>,<WS>,<adc_fq>,<log2_n>,[<rH1>],...,[<rH((2^log2_n)/2)>]}
//    _AV03 = 0x0E, // (14) FFT Real { <ET>,<WS>,<adc_fq>,<log2_n>,[<rH1>],...,[<rH((2^log2_n)/2)>]}
//    _AV04 = 0x04, // (04) Aeolian Vibration, RAW without dT ! { <ET>,<WS>,<adc_fq> <res_scale>,<s1>,...,<sn>} 
//    _AV05 = 0x0F, // (15) AVC-P2P { <ET>,<WS>,<adc_fq>,<res_scale>,<duration>,[ (<n>,<freq>,<amp>),...]}
//    _AV06 = 0x08, // (08) AVC-DFT { <ET>,<WS>,<adc_fq>,<res_scale>,<duration>,[ (<n>,<nc>,<pw>),...]}
//    _SS00 = 0x0B // (11) Sub-Span, raw    


#endif	/* XC_HEADER_TEMPLATE_H */

