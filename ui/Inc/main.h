

#include "stm32_ub_tim2.h"
#include "morse.h"
void SendFreq();
void FrequShift(int32_t freqShift);
extern float CWupper,CWlower,norm_sig, SigDiff,norm_avg,norm_noise,norm_peak,norm_value,sig_avg,agc_peak, agc_factor;
extern char sign;
extern uint32_t smpl_ctr;
extern uint16_t positx,posity;
extern volatile int cw_receive_state, cw_dot_length, CW_TIMING_STATE;
extern uint8_t UALabel;
