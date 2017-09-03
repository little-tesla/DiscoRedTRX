#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <pthread.h>
#include <termios.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <alsa/asoundlib.h>
#include <fftw3.h>

#include <linux/sched.h>
#include "comm.h"

#define I2C_SLAVE       0x0703 /* Use this slave address */
#define I2C_SLAVE_FORCE 0x0706 /* Use this slave address, even if it
                                  is already in use by a driver! */

#define ADDR_CODEC 0x1A /* WM8731 or TLV320AIC23B address 0 */

#define KeyUp 86
#define KeyDown 89
#define Key 82

enum cw_receive_state {
	IDLE,
	AREA1,
	AREA2
};

union {
int32_t dataA;
uint8_t bData[4];
} unionA;


const float Log2c[16]={0.0,0.09,0.17,0.25,0.32,0.39,0.46,0.52,0.58,0.64,0.70,0.75,0.81,0.86,0.91,0.95};
volatile float *rx_data, *tx_data;
volatile uint16_t *rx_cntr, *tx_cntr, *dac_cntr, *adc_cntr;
volatile uint8_t *rx_rst, *sp_rst, *tx_rst, *codec_rst;
volatile int32_t *dac_data, *adc_data;
volatile int32_t *win_data, *fft_data;
volatile int32_t *xadc;
volatile int Sema1;// Semaphore, 1: "CW Data transmission in progress"
volatile int Sema2;// Semaphore, 1: "CW Data provided"
volatile int32_t buffer3[256];
volatile uint8_t buffer[6],  buffer1[6];
volatile uint8_t oldcounterRx=0, counterRx=0;
volatile int32_t Pointerj;
volatile int32_t PointerA;
volatile uint8_t FFT_data_rdy=0;
uint8_t help;
volatile uint8_t FFT_new, FFT=0;
volatile uint8_t FFT_step=0;//"ready for new Pixels"
int8_t Analyzer_exists=0;// 1 if XCreateAnalyzer was successful
pthread_t thread1,thread2;
int32_t  cntr;
uint16_t timer, tick, counter2=0;
double rxbuffer0[512];
double rxbuffer1[512];
double rxaudiobuffer[256];
uint8_t Det_Mode=2; 
uint8_t Avg_Mode=3;
int FFT_Length_new=1<<13, FFT_Length=1<<13;//2^(5+8)=8192
uint16_t countU=0;
uint32_t wert1;
int i2c_codec = 0, j=0, indx=0, mode;
int uart_fd;
uint32_t CW_ThresholdLow;
uint32_t CW_ThresholdHigh;
float sig_avg, agc_factor, flat_value;
int cw_receive_state;
uint16_t counter, counter1,counterf;
int32_t NF1;
int32_t i, value, num;
uint8_t wert;
uint16_t r;
int flp[] = {1};
int result;
int ErrorCode= 0;//BuildNumber 14  /* Build Number */
pthread_mutex_t mutex1;//, mutex2;

//pthread_attr_t tattr;
//pthread_t tid;
//int ret;
//int newprio = 20;
//struct sched_param param;


ssize_t i2c_write8(int fd, uint8_t addr, uint8_t data)
{
  uint8_t ibuffer[2];
  ibuffer[0] = addr;
  ibuffer[1] = data;
  return write(fd, ibuffer, 2);
}

void *tx_data_handler(void *arg);
void *rx_data_handler(void *arg);
float log2x(int32_t x);
uint8_t Threshold3(int32_t value);
void SendUart(uint8_t scommand, uint32_t sdata);
void InitFFT(uint8_t FFT_type);

int main()
{
  int fd, i2c_fd, running, filter, ptt, mux,i,j,k,l, size, indexj;
  uint16_t r;
  FILE *wisdom_file;
  volatile void *cfg, *sts;
  volatile uint32_t *sp_phase, *tx_phase, *dac_phase, *alex, *tx_mux, *dac_mux;
  volatile int32_t *tx_ramp, *dac_ramp;
  volatile uint16_t *tx_size, *dac_size, *sp_rate, *sp_pre, *sp_tot, *sp_cntr;
  volatile uint16_t *tx_level, *dac_level;
  volatile uint8_t *gpio_in, *gpio_out;
  float scale, ramp[1024], a[4] = {0.35875, 0.48829, 0.14128, 0.01168};
  unsigned long num = 0;
  struct termios tty;
  uint8_t rbuffer[6];
  uint8_t code;
  uint32_t data;
  uint8_t crc8;
  float pixeldBm[8192];
  int	rc;
  uint8_t breaker;
  int32_t sum;
  int32_t rx_freq = 7200000;
  int32_t tx_freq = 7200000;
  int32_t shift;
  volatile uint32_t *rx_phase;// *rx_phase
  uint8_t volume;
  uint8_t agc;
  long min, max;
  int count=0;
  snd_mixer_t *handle;
  snd_mixer_selem_id_t *sid;
  snd_mixer_elem_t *playback, *capture;
  double cutoff[6][10][2] = {
    {{-613, -587}, {-625, -575}, {-650, -550}, {-725, -475}, {-800, -400}, {-850, -350}, {-900, -300}, {-975, -225}, {-1000, -200}, {-1100, -100}},
    {{587, 613}, {575, 625}, {550, 650}, {475, 725}, {400, 800}, {350, 850}, {300, 900}, {225, 975}, {200, 1000}, {100, 1100}},
    {{-1150, -150}, {-1950, -150}, {-2250, -150}, {-2550, -150}, {-2850, -150}, {-3050, -150}, {-3450, -150}, {-3950, -150}, {-4550, -150}, {-5150, -150}},
    {{150, 1150}, {150, 1950}, {150, 2250}, {150, 2550}, {150, 2850}, {150, 3050}, {150, 3450}, {150, 3950}, {150, 4550}, {150, 5150}},
    {{-2500, 2500}, {-3000, 3000}, {-3500, 3500}, {-4000, 4000}, {-4500, 4500}, {-5000, 5000}, {-6000, 6000}, {-8000, 8000}, {-9000, 9000}, {-12500, 12500}},
	{{-2500, 2500}, {-3000, 3000}, {-3500, 3500}, {-4000, 4000}, {-4500, 4500}, {-5000, 5000}, {-6250, 6250}, {-8000, 8000}, {-9000, 9000}, {-12500, 12500}}};


  if((fd = open("/dev/mem", O_RDWR)) < 0)
  {
    perror("open");
    return EXIT_FAILURE;
  }

  if((i2c_fd = open("/dev/i2c-0", O_RDWR)) >= 0)
  {
    if(ioctl(i2c_fd, I2C_SLAVE_FORCE, ADDR_CODEC) >= 0)
    {
      /* reset */
      if(i2c_write8(i2c_fd, 0x1e, 0x00) > 0)
      {
        i2c_codec = 1;
        /* set power down register */
        i2c_write8(i2c_fd, 0x0c, 0x51);
        /* reset activate register */
        i2c_write8(i2c_fd, 0x12, 0x00);
        /* set volume to -30 dB */
        i2c_write8(i2c_fd, 0x04, 0x5b);
        i2c_write8(i2c_fd, 0x06, 0x5b);
        /* set analog audio path register */
        i2c_write8(i2c_fd, 0x08, 0x15);
        /* set digital audio path register */
        i2c_write8(i2c_fd, 0x0a, 0x00);
        /* set format register */
        i2c_write8(i2c_fd, 0x0e, 0x42);
        /* set activate register */
        i2c_write8(i2c_fd, 0x12, 0x01);
        /* set power down register */
        i2c_write8(i2c_fd, 0x0c, 0x41);
      }
    }
  }

  if((uart_fd = open("/dev/ttyPS1", O_RDWR|O_NOCTTY|O_NDELAY)) < 0)
  {
    perror("open");
    return EXIT_FAILURE;
  }

  tcgetattr(uart_fd, &tty);
  cfsetspeed(&tty, (speed_t)B115200);
  cfmakeraw(&tty);
  tty.c_cflag &= ~(CSTOPB | CRTSCTS);
  tty.c_cflag |= CLOCAL | CREAD;
  tcflush(uart_fd, TCIFLUSH);
  tcsetattr(uart_fd, TCSANOW, &tty);

  if(!i2c_codec)
  {
    snd_mixer_open(&handle, 0);
    snd_mixer_attach(handle, "default");
    snd_mixer_selem_register(handle, NULL, NULL);
    snd_mixer_load(handle);

    snd_mixer_selem_id_alloca(&sid);
    snd_mixer_selem_id_set_index(sid, 0);
    snd_mixer_selem_id_set_name(sid, "Speaker");
    playback = snd_mixer_find_selem(handle, sid);

    snd_mixer_selem_id_alloca(&sid);
    snd_mixer_selem_id_set_index(sid, 0);
    snd_mixer_selem_id_set_name(sid, "Mic");
    capture = snd_mixer_find_selem(handle, sid);
  }

  sts = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40000000);
  cfg = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40001000);
  alex = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40002000);
  rx_data = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40003000);
  tx_data = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40005000);
  tx_ramp = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40006000);
  tx_mux = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40007000);
  win_data = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40008000);
  fft_data = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40009000);
  dac_data = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x4000A000);
  dac_ramp = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x4000B000);
  dac_mux = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x4000C000);
  adc_data = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x4000D000);
  xadc = mmap(NULL, 16*sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0x40020000);

  rx_rst = ((uint8_t *)(cfg + 0));
  sp_rst = ((uint8_t *)(cfg + 0));
  tx_rst = ((uint8_t *)(cfg + 1));
  codec_rst = ((uint8_t *)(cfg + 2));
  gpio_out = ((uint8_t *)(cfg + 3));

  rx_phase = ((uint32_t *)(cfg + 4));

  sp_rate = ((uint16_t *)(cfg + 12));
  sp_phase = ((uint32_t *)(cfg + 16));
  sp_pre = ((uint16_t *)(cfg + 20));
  sp_tot = ((uint16_t *)(cfg + 22));

  tx_phase = ((uint32_t *)(cfg + 24));
  tx_size = ((uint16_t *)(cfg + 28));
  tx_level = ((uint16_t *)(cfg + 30));

  dac_phase = ((uint32_t *)(cfg + 32));
  dac_size = ((uint16_t *)(cfg + 36));
  dac_level = ((uint16_t *)(cfg + 38));

  rx_cntr = ((uint16_t *)(sts + 12));
  sp_cntr = ((uint16_t *)(sts + 18));
  tx_cntr = ((uint16_t *)(sts + 20));
  dac_cntr = ((uint16_t *)(sts + 22));
  adc_cntr = ((uint16_t *)(sts + 24));
  gpio_in = ((uint8_t *)(sts + 26));

  /* set PTT pin to low */
  *gpio_out = 0;

  if((wisdom_file = fopen("wdsp-fftw-wisdom.txt", "r")))
  {
    fftw_import_wisdom_from_file(wisdom_file);
    fclose(wisdom_file);
  }

  OpenChannel(0, 256, 2048, 48000, 48000, 48000, 0, 0, 0.010, 0.025, 0.000, 0.010, 0);
  OpenChannel(1, 256, 2048, 48000, 48000, 48000, 1, 0, 0.010, 0.025, 0.000, 0.010, 0);

  if((wisdom_file = fopen("wdsp-fftw-wisdom.txt", "w")))
  {
    fftw_export_wisdom_to_file(wisdom_file);
    fclose(wisdom_file);
  }

  SetRXAShiftRun(0, 0);
  SetRXAPanelGain1(0, 1.0);
  SetRXAAGCMode(0, 3);
  RXASetNC(0, 2048);
  RXASetMP(0, 1);

  SetTXACompressorRun(1, 1);
  SetPSRunCal(1, 0);
  TXASetNC(1, 2048);
  TXASetMP(1, 1);

  mode = 3;
  shift = 0;
  filter = 4;

  *rx_phase = (uint32_t)floor((rx_freq + shift) / 125.0e6 * (1<<30) + 0.5);
  *tx_phase = (uint32_t)floor(tx_freq / 125.0e6 * (1<<30) + 0.5);

  SetRXAMode(0, RXA_LSB);
  SetTXAMode(1, TXA_LSB);

  RXASetPassband(0, cutoff[mode][filter][0], cutoff[mode][filter][1]);
  SetTXABandpassFreqs(1, cutoff[mode][filter][0], cutoff[mode][filter][1]);

  /* set tx ramp */
  size = 1001;
  ramp[0] = 0.0;
  for(i = 1; i <= size; ++i)
  {
    ramp[i] = ramp[i - 1] + a[0] - a[1] * cos(2.0 * M_PI * i / size) + a[2] * cos(4.0 * M_PI * i / size) - a[3] * cos(6.0 * M_PI * i / size);
  }
  scale = 6.1e6 / ramp[size];
  for(i = 0; i <= size; ++i)
  {
    tx_ramp[i] = (uint32_t)floor(ramp[i] * scale + 0.5);
  }
  *tx_size = size;

  /* set default tx level */
  *tx_level = 0;

  /* set default tx mux channel */
  *(tx_mux + 16) = 0;
  *tx_mux = 2;

  *rx_rst |= 1;
  *rx_rst &= ~1;

  *sp_rst &= ~2;
  *sp_rst |= 4;
  *sp_rst &= ~4;
  *sp_rst |= 2;

  *tx_rst |= 1;
  *tx_rst &= ~1;

  if(i2c_codec)
  {
    /* reset codec fifo buffers */
    *codec_rst |= 3;
    *codec_rst &= ~3;
    /* enable I2S interface */
    *codec_rst &= ~4;

    /* set default dac phase increment */
    *dac_phase = (uint32_t)floor(600 / 48.0e3 * (1 << 30) + 0.5);

    /* set dac ramp */
    size = 481;
    ramp[0] = 0.0;
    for(i = 1; i <= size; ++i)
    {
      ramp[i] = ramp[i - 1] + a[0] - a[1] * cos(2.0 * M_PI * i / size) + a[2] * cos(4.0 * M_PI * i / size) - a[3] * cos(6.0 * M_PI * i / size);
    }
    scale = 3.2e4 / ramp[size];
    for(i = 0; i <= size; ++i)
    {
      dac_ramp[i] = (int32_t)floor(ramp[i] * scale + 0.5);
    }
    *dac_size = size;

    /* set default dac level */
    *dac_level = 32767;

    /* set default dac mux channel */
    *(dac_mux + 16) = 0;
    *dac_mux = 2;
  }
  else
  {
    /* enable ALEX interface */
    *codec_rst |= 4;
  }

  SetChannelState(0, 1, 0);
  SetChannelState(1, 1, 0);

  if(pthread_create(&thread1, NULL, rx_data_handler, NULL) < 0)
  {
    perror("pthread_create_rx");
    return EXIT_FAILURE;
  }
  pthread_detach(thread1);//(thread1);

  if(pthread_create(&thread2, NULL, tx_data_handler, NULL) < 0)
  {
    perror("pthread_create_tx");
    return EXIT_FAILURE;
  }
  pthread_detach(thread2);
  
  InitCW();
  SendUart(60,ErrorCode);
  running = 1;
  FFT=0;
  i=0;indexj=0;
  FFT_step=0;
  FFT_data_rdy=0;	
  while(running)
  {		
	if(FFT!=0){		
		if(FFT_data_rdy!=0){// rf	
			FFT_data_rdy=0;
			if(FFT==1)
				Spectrum0(1,0,0,0,rxbuffer0);//calculate FFT for RF
			else if(FFT==2){// audio
				for(i=0;i<256;i++){
					rxaudiobuffer[i]=0.0003*rxbuffer1[2*i];
				}	
				Spectrum0(1,0,0,0,rxaudiobuffer);//calculate FFT for AF
			}
			rc=0;
			if(FFT>=1)
				GetPixels(0,0,&pixeldBm[0],&rc);//test, if rf pixel values	present	
			
			if(rc) {
				SendUart(86, 0x10204080);//send start of line	
				FFT_step=1;
				indexj=0;
			}
		}
		else usleep(1000);	
		switch(FFT_step){
			case 0:							
				usleep(1000);
				break;
			case 1:									
					/*for(k=0;k<480;k++){
							l++;
							if(l>=150) l=0;        //test: sliding stairway
							pixeldBm[k]=l/2.0+5.0;
					}*/
					
				while(indexj<480){
					usleep(100);//1000
					if((indexj&3)==0)
						sum=(uint8_t)(int32_t)(140.0+pixeldBm[indexj]);
					else						
						sum+=(uint8_t)(int32_t)(140.0+pixeldBm[indexj]);
					if((indexj&3)!=3){
						sum*=256;					
					}				
					else SendUart(100, sum);//send 4 bytes data
					indexj++;
					ioctl(uart_fd, FIONREAD, &num);// test for UART input
					if(num!=0) {
						break;
					}	
				}
				FFT_step=0;
				break;
			}		
	}
	  	  
	if(mode<2){//&&(FFT==0)) {
		while(Sema2 ==0) usleep(2000);//wait for data in buffer3 (period 5,33 ms) 
		Sema1=0;
		NF1=0;// 600 Hz filter
		for(i = 40; i < 120; i ++){// (i = 40; i < 80; i ++) interval [x, x+2*pi]
			NF1+=(buffer3[i-40]/2097152)*(buffer3[i]/2097152);//2^^18  sin(x)=-sin(pi+x)  262144
			//folding with itself, expected result is < 0 (for sinewave)
		}
		if(NF1<0) NF1 =-NF1;
		Sema1=1;
		wert=Threshold3(NF1);//NF1: filtered NF sum value
		if((wert!=0)||(++counter%20==0)){// all 107 ms
			if(wert!=0)
				SendUart(wert, flat_value);	// 86 KeyUp // 89 KeyDown // 82 Key
			else
				SendUart(82, flat_value);// submit only the value
		}
	}
	else if(FFT==0)	{
		usleep(40000);//45000
	}
	tick++;
	if(tick!=timer) {
		tick=timer;
		counter2++;
		if(counter2>=50){//period: 266,5 ms
			counter2=0;
			SendUart(85, floor(-10.0 * GetRXAMeter(0, 1) + 0.5));// S-Value
		}
	}
    while(1)
    {
      ioctl(uart_fd, FIONREAD, &num);
      if(num < 6) break;
      read(uart_fd, rbuffer, 6);
      code = *(uint8_t *)(rbuffer + 0);
      data = *(uint32_t *)(rbuffer + 1);
      crc8 = *(uint8_t *)(rbuffer + 5);
      switch(code)
      {
        case 1:
          if(data > 62000000) continue;
          shift=0;
          if (mode==0) shift=-600;
          else if (mode==1) shift = 600;
          rx_freq = data;
          *rx_phase = (uint32_t)floor((rx_freq + shift) / 125.0e6 * (1<<30) + 0.5);
          break;
        case 2:
          if(data > 62000000) continue;
          tx_freq = data;
          *tx_phase = (uint32_t)floor(tx_freq / 125.0e6 * (1<<30) + 0.5);
          break;
        case 3:// output volume
          if(data > 40) continue;
          if(i2c_codec)
          {
            //volume = 48 + data * 7;
			volume = 47 + data * 2;// 41 steps each 2 dB ( 0..40 -> 47..127) // 47 = mute
            i2c_write8(i2c_fd, 0x04, volume);
            i2c_write8(i2c_fd, 0x06, volume);
          }
          else
          {
            snd_mixer_selem_get_playback_volume_range(playback, &min, &max);
            //snd_mixer_selem_set_playback_volume_all(playback, data * max / 9);
            snd_mixer_selem_set_playback_volume_all(playback, data * max / 40);
          }
          break;
        case 4:// Microphone volume
          if(data > 30) continue;
          if(i2c_codec)
          {
			volume = data ;// 31 steps each 1.5 dB // 128 = mute
			if(volume==0) volume=128;// mute
			i2c_write8(i2c_fd, 0x00, volume);
            i2c_write8(i2c_fd, 0x02, volume);
          }
          else
          {
            snd_mixer_selem_get_capture_volume_range(capture, &min, &max);
            snd_mixer_selem_set_capture_volume_all(capture, data * max / 30);
          }
          break;
        case 5:
          if(data > 9) continue;
          filter = data;
          RXASetPassband(0, cutoff[mode][filter][0], cutoff[mode][filter][1]);
          SetTXABandpassFreqs(1, cutoff[mode][filter][0], cutoff[mode][filter][1]);
          break;
        case 6:
          if(data > 7) continue;// WK
          mode = data;
          shift = 0;
          switch(mode)
          {
            case 0:
              shift = -600;// 600
              SetRXAMode(0, RXA_CWL);
              SetTXAMode(1, TXA_CWL);
              InitCW();
              break;
            case 1:
              shift = 600;
              SetRXAMode(0, RXA_CWU);
              SetTXAMode(1, TXA_CWU);
              InitCW();
              break;
            case 2:
              SetRXAMode(0, RXA_LSB);
              SetTXAMode(1, TXA_LSB);
              break;
            case 3:
              SetRXAMode(0, RXA_USB);
              SetTXAMode(1, TXA_USB);
              break;
            case 4:
              SetRXAMode(0, RXA_AM);
              SetTXAMode(1, TXA_AM);
              break;
			case 5://  new 29.04.2017 WK
              SetRXAMode(0, RXA_FM);
              SetTXAMode(1, TXA_FM);
              break;
			case 7:// SAM
			  mode=5;// filters: like AM
			  SetRXAMode(0, RXA_SAM);
			  SetTXAMode(1, TXA_AM);
			  break;
          }
          *rx_phase = (uint32_t)floor((rx_freq + shift) / 125.0e6 * (1<<30) + 0.5);
          RXASetPassband(0, cutoff[mode][filter][0], cutoff[mode][filter][1]);
          SetTXABandpassFreqs(1, cutoff[mode][filter][0], cutoff[mode][filter][1]);
          break;
        case 7:
			if(data > 4) continue;
			agc = data;
			SetRXAAGCMode(0, agc);
			break;
        case 8:
          if(data > 1) continue;
          ptt = data;
          *gpio_out = data;
          *tx_level = data ? 32767 : 0;
          break;
        case 9:
          running = 0;
          break;
		case 10:
			SendUart(60,ErrorCode);
			break;
		case 11:
			if(mode>1) continue;
			if((data>10000)||(data<100)) continue;
			CW_ThresholdHigh=data;
			break;
		case 12:
			if(mode>1) continue;
			if((data>400)||(data<5)) continue;
			CW_ThresholdLow=data;
			break;
		case 13:
			if(data==0)
				SetRXAANFRun(0,0);
			else{
				SetRXAANFPosition(0,0);
				SetRXAANFRun(0,1);
			}
			break;
		case 14:
			if(data==0)
				SetRXASNBARun(0,0);
			else{
				SetRXASNBARun(0,1);
			}
			break;
		case 15:
			if(data==0)
				SetRXAEMNRRun(0,0);
			else{
				SetRXAEMNRRun(0,1);
			}
			break;
		case 16:
			if(data==0)
				SetRXAANRRun(0,0);
			else{
				SetRXAANRRun(0,1);
			}
			break;
		case 17:
			unionA.dataA=data;
			FFT_new=unionA.bData[0];
			if((FFT_new>=3)||(FFT_new==0)) {
				FFT=FFT_new=0;
				if(Analyzer_exists>=1){
					DestroyAnalyzer(0);
					Analyzer_exists--;
				}
				break;
			}
			Det_Mode=unionA.bData[1];
			Avg_Mode=unionA.bData[2]-1;//values: -1 .. 3
			if(FFT_new==1){								
				if(unionA.bData[3]>5) unionA.bData[3]=5;
					FFT_Length_new=1<<(unionA.bData[3]+8);
			}
			else if (FFT_new==2){
				if(unionA.bData[3]>2) unionA.bData[3]=2;
					FFT_Length_new=1<<(unionA.bData[3]+8);
			}
			InitFFT(FFT_new);
			FFT_step=0;
			FFT_data_rdy=0;		
      }
      data = ptt & (mode < 2);
      if(mux != data)
      {
        mux = data;
        *(tx_mux + 16) = data;
        *tx_mux = 2;
        if(i2c_codec)
        {
          *(dac_mux + 16) = data;
          *dac_mux = 2;
        }
      }
    }       
  }

  /* set PTT pin to low */
  *gpio_out = 0;
 
  return EXIT_SUCCESS; 
}

float log2x(int32_t x){// Logarithm Base 2
uint32_t y;
int8_t z;

	if(x<=0) return -1;
	y=x;
	z=4;
	while(y<16){
		z--;
		y=y<<1;
	}
	for(;y>16;z++){
		y=y>>1;
	}
	return (Log2c[y] + z);
}

void *rx_data_handler(void *arg)
{
int32_t i ,j , value, error,result;

float buffer2[512];
 
  Pointerj=0;
  cntr=0;
  while(1)
  {
    if(*rx_cntr >= 1024)
    {
      *rx_rst |= 1;
      *rx_rst &= ~1;
    }
    FFT_data_rdy++;
	while(*rx_cntr < 512) {//512		
		usleep(1000);//100
	}
	Pointerj=0;
	timer++;
	while(Pointerj<512){				
		rxbuffer0[Pointerj++] =(double) *rx_data;
	}
	fexchange0(0, rxbuffer0, rxbuffer1, &error);
	if(i2c_codec)
	{

	  while(*dac_cntr > 256) usleep(1000);
	  if(*dac_cntr == 0) for(i = 0; i < 256; ++i) {
		  *dac_data = 0;
	  }
	  if(mode<2){
		  Sema2=0;
		  j=0;
		  for(i = 0; i < 512; i += 2){
				value = (int32_t)(rxbuffer1[i] * 32766.0 ) << 16;
				value |= (int32_t)(rxbuffer1[i + 1]* 32766.0) & 0xffff;
				*dac_data = value;
				buffer3[j++]=value;
		  }
		  Sema2=1;// "data ready"
	  }
	  else{// mode >=2
		  for(i = 0; i < 512; i += 2){
			value = (int32_t)(rxbuffer1[i] * 32766.0 ) << 16 ;
			value |= (int32_t)(rxbuffer1[i + 1] * 32766.0) & 0xffff;//
			*dac_data = value;
		  }
	  }
	}

	else    {// (no codec)
		Sema2=0;
		j=0;
		for(i = 0; i < 512; ++i) {
			buffer2[i] = rxbuffer1[i];
			if((i&1)==0)
				buffer3[j++]=(int32_t)buffer2[i];
		}
		Sema2=1;// "data ready"
		fwrite(buffer2, 4, 512, stdout);
		fflush(stdout);
	}
  }
  return NULL;
}

void *tx_data_handler(void *arg)
{
  int32_t i, error, value;
  float buffer0[512];
  double buffer1[512], buffer2[512];

  while(1)
  {
    while(*tx_cntr > 512) usleep(1000);

    if(*tx_cntr == 0)
    {
      for(i = 0; i < 512; ++i) *tx_data = 0.0;
    }

    if(i2c_codec)
    {
      if(*adc_cntr >= 512)
      {
        *codec_rst |= 2;
        *codec_rst &= ~2;
      }
      while(*adc_cntr < 256) usleep(1000);
      for(i = 0; i < 512; i += 2)
      {
        value = *adc_data;
        buffer1[i] = (double)(value >> 16) / 32767.0;
        buffer1[i + 1] = (double)(value & 0xffff) / 32767.0;
      }
    }
    else
    {
      fread(buffer0, 4, 512, stdin);
      for(i = 0; i < 512; ++i) buffer1[i] = buffer0[i];
    }

    fexchange0(1, buffer1, buffer2, &error);

    for(i = 0; i < 512; ++i) *tx_data = buffer2[i];
  }
  return NULL;
}

void InitCW(void){
	cw_receive_state = IDLE;
	sig_avg=0;
	agc_factor=0.0002;
	CW_ThresholdLow=400;//20 // war 40  / 80 /200
	CW_ThresholdHigh=1200;// 200 /1000 /600
}

float moving_average(float average, float input, int weight)
{
	return ( ( input - average ) / weight  + average) ;
}

uint8_t Threshold3(int32_t value) {
float agc_delta;

	flat_value=agc_factor*value;								//*** gain control ***
	sig_avg = moving_average(sig_avg, flat_value, 400);// 1000 200
	if(cw_receive_state == IDLE){//||((agc_delta>=5.0)||(agc_delta<=-5.0)))
		agc_delta=sig_avg-100.0;// setpoint = 25  / 100
		if(agc_delta>5)agc_delta=5.0;
		if(agc_delta<-5)agc_delta=-5.0;
		agc_factor*=1.0-agc_delta*0.000003;
	}
	else if(cw_receive_state == AREA2){
		agc_delta=sig_avg-100.0;// setpoint = 200
		if(agc_delta>20)agc_delta=20.0;
		if(agc_delta<-20)agc_delta=-20.0;
		agc_factor*=1.0-agc_delta*0.0000009;// 0.0000001
	}
	if(flat_value > CW_ThresholdHigh){// 200
		if	(cw_receive_state != AREA2)  {
			cw_receive_state = AREA2;
			return KeyDown;
		}
		else return 0;
	}
	else if (flat_value < CW_ThresholdLow){// 20

		if(cw_receive_state != IDLE){
			cw_receive_state = IDLE;
			return KeyUp;
		}
	}
	else if(cw_receive_state != AREA1){// between Low and High
		cw_receive_state = AREA1;
		return Key;
	}
	return 0;
}
void SendUart(uint8_t scommand, uint32_t sdata){
uint8_t	sbuffer[6];
int32_t snum;

ioctl(uart_fd, TIOCOUTQ, &snum);
while (snum!=0){
	usleep(1000);
	ioctl(uart_fd, TIOCOUTQ, &snum);// wait for end of transmission
}
*(uint8_t *)(sbuffer + 0) = scommand;
*(uint32_t *)(sbuffer + 1) = sdata;
*(uint8_t *)(sbuffer + 5) = 160;
write(uart_fd, sbuffer, 6);
}

void InitFFT(uint8_t FFT_type){
int data_type;
int result;

if((FFT_type<1)||(FFT_type>2))	return;
if((FFT_type!=FFT)||(FFT_Length_new!=FFT_Length)) {
		FFT_Length=FFT_Length_new;
		if (Analyzer_exists>0) {
			DestroyAnalyzer(0);
			Analyzer_exists--;
		}
		if (Analyzer_exists<=0){
			XCreateAnalyzer ( 0,//display number (0..63)
					  &result,// pointer to result of creation
					  FFT_Length,//max. FFT width 8192
					  1,//max_LO
					  1,//max sub-spans
					  "");
			if(result==0) Analyzer_exists=1;
			else return;
		}
		FFT=FFT_type;
		if(FFT_type==1) data_type=1;// RF
		else data_type=0;//AF
		
		SetAnalyzer(0,//disp
			1,//n_pixout
			1, //number of LO frequencies = number of ffts used in elimination
			data_type, //data type 0 for real input data (I only); 1 for complex input data (I & Q)
			flp, //vector with one elt for each LO frequency, 1 if high-side LO, 0 otherwise
			FFT_Length,//2048,4096 8192//256 size of the fft, i.e., number of input samples   war 256					XXXXXX 8192
			256,//256, //number of samples transferred for each OpenBuffer()/CloseBuffer() ****
			5, //integer specifying which window function to use
			14.0, //PiAlpha parameter for Kaiser window
			FFT_Length/4,//overlap, //number of samples each fft (other than the first) is to re-use from the previous
			0,//clip, //number of fft output bins to be clipped from EACH side of each sub-span
			0,//span_clip_l, //number of bins to clip from low end of entire span
			0,//span_clip_h, //number of bins to clip from high end of entire span
			512,//pixels, //number of pixel values to return.  may be either <= or > number of bins ****
			1,//stitches, //number of sub-spans to concatenate to form a complete span
			0,//calibration_data_set, //identifier of which set of calibration data to use
			0.0,//span_min_freq, //frequency at first pixel value
			0.0,//span_max_freq, //frequency at last pixel value
			2*FFT_Length//max_w //max samples to hold in input ring buffers  war 8192
			);
	}
	SetDisplayDetectorMode(0,//display identity
			0,//pixout(channel)
			Det_Mode);//average mode
	SetDisplayAverageMode(0,//disp
			0,//pixout  
			Avg_Mode);// weighted average	
	SetDisplayAvBackmult(0, 0, 0.85);
	if(FFT==1)
		SetDisplayNumAverage(0, 0, 40);
	else 
		SetDisplayNumAverage(0, 0, 10);
}
