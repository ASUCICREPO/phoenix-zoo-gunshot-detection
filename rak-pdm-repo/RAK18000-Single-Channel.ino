
#include <PDM.h>
#include "arduinoFFT.h"
#include <Arduino.h>
#include <LoRaWan-RAK4630.h>
#include <bluefruit.h>

#define PIN_VBAT WB_A0

uint32_t vbat_pin = PIN_VBAT;

#define VBAT_MV_PER_LSB (0.87890625) // 3.0V ADC range and 12 - bit ADC resolution = 3000mV / 4096
#define VBAT_DIVIDER_COMP (1.48)      // Compensation factor for the VBAT divider, depend on the board

#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)

float readVBAT(void)
{
    float raw;

    // Get the raw 12-bit, 0..3000mV ADC value
    raw = analogRead(vbat_pin);

    return raw * REAL_VBAT_MV_PER_LSB;
}


// RAK4630 supply two LED
#ifndef LED_BUILTIN
#define LED_BUILTIN 35
#endif

#ifndef LED_BUILTIN2
#define LED_BUILTIN2 36
#endif

bool doOTAA = true;   // OTAA is used by default.
#define SCHED_MAX_EVENT_DATA_SIZE APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE 60                      /**< Maximum number of events in the scheduler queue. */
#define LORAWAN_DATERATE DR_0                   /*LoRaMac datarates definition, from DR_0 to DR_5*/
#define LORAWAN_TX_POWER TX_POWER_5             /*LoRaMac tx power definition, from TX_POWER_0 to TX_POWER_15*/
#define JOINREQ_NBTRIALS 3                      /**< Number of trials for the join request. */
DeviceClass_t g_CurrentClass = CLASS_A;         /* class definition*/
LoRaMacRegion_t g_CurrentRegion = LORAMAC_REGION_US915;    /* Region:EU868*/
lmh_confirm g_CurrentConfirm = LMH_CONFIRMED_MSG;         /* confirm/unconfirm packet definition*/
uint8_t gAppPort = LORAWAN_APP_PORT;                      /* data port*/

/**@brief Structure containing LoRaWan parameters, needed for lmh_init()
*/
static lmh_param_t g_lora_param_init = {LORAWAN_ADR_ON, LORAWAN_DATERATE, LORAWAN_PUBLIC_NETWORK, JOINREQ_NBTRIALS, LORAWAN_TX_POWER, LORAWAN_DUTYCYCLE_OFF};

// Foward declaration
static void lorawan_has_joined_handler(void);
static void lorawan_join_failed_handler(void);
static void lorawan_rx_handler(lmh_app_data_t *app_data);
static void lorawan_confirm_class_handler(DeviceClass_t Class);
static void send_lora_frame(void);

/**@brief Structure containing LoRaWan callback functions, needed for lmh_init()
*/
static lmh_callback_t g_lora_callbacks = {BoardGetBatteryLevel, BoardGetUniqueId, BoardGetRandomSeed,
                                        lorawan_rx_handler, lorawan_has_joined_handler, lorawan_confirm_class_handler, lorawan_join_failed_handler
                                       };
//OTAA keys !!!! KEYS ARE MSB !!!!
uint8_t nodeDeviceEUI[8] = {0xAC, 0x1F, 0x09, 0xFF, 0xFE, 0x03, 0xBA, 0x1C}; // Enter DevEUI
uint8_t nodeAppEUI[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; // Enter AppEUI
uint8_t nodeAppKey[16] = {0xF6, 0xD9, 0xE3, 0x21, 0x36, 0x31, 0x5B, 0x94, 0x45, 0x8D, 0xFE, 0x47, 0x2B, 0x25, 0x20, 0x02}; // Enter AppKey

// ABP keys
uint32_t nodeDevAddr = 0x260116F8;
uint8_t nodeNwsKey[16] = {0x7E, 0xAC, 0xE2, 0x55, 0xB8, 0xA5, 0xE2, 0x69, 0x91, 0x51, 0x96, 0x06, 0x47, 0x56, 0x9D, 0x23};
uint8_t nodeAppsKey[16] = {0xFB, 0xAC, 0xB6, 0x47, 0xF3, 0x58, 0x45, 0xC7, 0x50, 0x7D, 0xBF, 0x16, 0x8B, 0xA8, 0xC1, 0x7C};

// Private defination
#define LORAWAN_APP_DATA_BUFF_SIZE 64                     /**< buffer size of the data to be transmitted. */
#define LORAWAN_APP_INTERVAL 10000                        /**< Defines for user timer, the application data transmission interval. 20s, value in [ms]. */
static uint8_t m_lora_app_data_buffer[LORAWAN_APP_DATA_BUFF_SIZE];            //< Lora user application data buffer.
static lmh_app_data_t m_lora_app_data = {m_lora_app_data_buffer, 0, 0, 0, 0}; //< Lora user application data structure.

TimerEvent_t appTimer;
static uint32_t timers_init(void);
static uint32_t count = 0;
static uint32_t count_fail = 0;

struct HEADER {
  unsigned char riff[4];            // RIFF string
  unsigned int overall_size ;       // overall size of file in bytes
  unsigned char wave[4];            // WAVE string
  unsigned char fmt_chunk_marker[4];      // fmt string with trailing null char
  unsigned int length_of_fmt;         // length of the format data
  unsigned int format_type;         // format type. 1-PCM, 3- IEEE float, 6 - 8bit A law, 7 - 8bit mu law
  unsigned int channels;            // no.of channels
  unsigned int sample_rate;         // sampling rate (blocks per second)
  unsigned int byterate;            // SampleRate * NumChannels * BitsPerSample/8
  unsigned int block_align;         // NumChannels * BitsPerSample/8
  unsigned int bits_per_sample;       // bits per sample, 8- 8bits, 16- 16 bits etc
  unsigned char data_chunk_header [4];    // DATA string or FLLR string
  unsigned int data_size;           // NumSamples * NumChannels * BitsPerSample/8 - size of the next chunk that will be read
};

unsigned char buffer4[4];
unsigned char buffer2[2];

struct HEADER header;

const uint16_t samples = 256;
double vReal[samples];
double vImag[samples];
//
//int myBins[20] = {
//  23, 69, 115, 184, 276, 368, 460, 621, 851, 1196, 1656, 2116,
//                  3496, 5796, 8096, 10396, 12696, 14996, 17296, 21223
//                 };

//int myBins[128]=0;

//int myBins[20] = {
//  8, 23, 40, 64, 96, 128, 160, 216, 296, 416, 576, 736,
//                  1216, 2016, 2816, 3616, 4416, 14996, 5216, 7382
//                 };




int fft_bins=samples/2;

double level[128];
double lastLevel[20];
double diffLevel[20];
double diffLevelSum;
double prevDiffLevelSum;

double SpectralCentroid;
double SpectralCentroidPrev;
double den;
double num;

int specDiff[3];

//int count=0;

long st;

short sampleBuffer[256];

//const uint16_t samples = 1024;
//double vReal[samples];
int j=0;

// number of samples read
volatile int samplesRead;

double samples_fft[samples];
boolean dont_change=false;

arduinoFFT FFT = arduinoFFT();

void detectShot(){
//   long millisecs=millis();
   arduinoFFT FFT = arduinoFFT();
   FFT.Windowing(vReal, samples, FFT_WIN_TYP_HANN, FFT_FORWARD);
   FFT.Compute(vReal, vImag, samples, FFT_FORWARD);
   FFT.ComplexToMagnitude(vReal, vImag, samples);

   for( int z=0; z<fft_bins; z++ ){
//    lastLevel[z]=level[z];
    level[z]=0;
  }
  // i < (samples >> 1)
  for (int i = 0; i<fft_bins ; i++) {
    vReal[i] = vReal[i] / samples;
    level[i]=vReal[i];
  }

  SpectralCentroidPrev = SpectralCentroid;
  num=0;
  den=0;
  
  for (int i = 0; i < fft_bins; i++) {
    
    num += (level[i]) * 62.5 * (i+1);
    den += (level[i]);
    
  }
  
  if(den==0) den=1;

  SpectralCentroid = (num / den);

  prevDiffLevelSum=diffLevelSum;
  diffLevelSum=0;
  
  for (int i = 0; i < 5; i++) {
    diffLevel[i] = (level[i] - lastLevel[i]);
    diffLevelSum += (diffLevel[i]);
    lastLevel[i] = level[i];
  }

  long amp = diffLevelSum-prevDiffLevelSum;
  long cen = SpectralCentroid;

//  specDiff[0]=specDiff[1];
//  specDiff[1]=specDiff[2];
//  specDiff[2]=SpectralCentroid-SpectralCentroidPrev;

  specDiff[0]=specDiff[1];
  specDiff[1]=specDiff[2];
  specDiff[2]=specDiff[3];
  specDiff[3]=specDiff[4];
  specDiff[4]=SpectralCentroid;
  
    boolean cen_check=false;
    for(int h=0; h<5; h++){
//      Serial.print(specDiff[h]);
//      Serial.print(" ");
//      
      if(specDiff[h]<200){
        cen_check=true;
        break;
      }
    }
//    Serial.print(SpectralCentroid);
//      Serial.print(" ");
//    Serial.println(200);
    
    if( (diffLevelSum>7500) && cen_check==true){
//      Serial.println("Gunshot detected");
//      cen_check=false;
      send_lora_frame();
//      delay(1000);
    }

//  long mils=millis()-millisecs;
//  Serial.println(mils);

//  delay(10000);

}



void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize LoRa chip.
  lora_rak4630_init();

  // Initialize Serial for debug output
  time_t timeout = millis();
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("=====================================");
  Serial.println("Welcome to RAK4630 LoRaWan!!!");
  if (doOTAA)
  {
    Serial.println("Type: OTAA");
  }
  else
  {
    Serial.println("Type: ABP");
  }

  switch (g_CurrentRegion)
  {
    case LORAMAC_REGION_AS923:
      Serial.println("Region: AS923");
      break;
    case LORAMAC_REGION_AU915:
      Serial.println("Region: AU915");
      break;
    case LORAMAC_REGION_CN470:
      Serial.println("Region: CN470");
      break;
    case LORAMAC_REGION_EU433:
      Serial.println("Region: EU433");
      break;
    case LORAMAC_REGION_IN865:
      Serial.println("Region: IN865");
      break;
    case LORAMAC_REGION_EU868:
      Serial.println("Region: EU868");
      break;
    case LORAMAC_REGION_KR920:
      Serial.println("Region: KR920");
      break;
    case LORAMAC_REGION_US915:
      Serial.println("Region: US915");
      break;
  }
  Serial.println("=====================================");
  
  //creat a user timer to send data to server period
  uint32_t err_code;
//  err_code = timers_init();
//  if (err_code != 0)
//  {
//    Serial.printf("timers_init failed - %d\n", err_code);
//    return;
//  }

  // Setup the EUIs and Keys
  if (doOTAA)
  {
    lmh_setDevEui(nodeDeviceEUI);
    lmh_setAppEui(nodeAppEUI);
    lmh_setAppKey(nodeAppKey);
  }
  else
  {
    lmh_setNwkSKey(nodeNwsKey);
    lmh_setAppSKey(nodeAppsKey);
    lmh_setDevAddr(nodeDevAddr);
  }

  // Initialize LoRaWan
  err_code = lmh_init(&g_lora_callbacks, g_lora_param_init, doOTAA, g_CurrentClass, g_CurrentRegion);
  Serial.printf("lmh_init response code: %d\n", err_code);
  if (err_code != 0)
  {
    Serial.printf("lmh_init failed - %d\n", err_code);
    return;
  }

  // Start Join procedure
  lmh_join();

  Serial.begin(115200);
  while (!Serial) {}

  PDM.setPins(24, 25, -1);

//  PDM.setPins(PDM_DATA_PIN, PDM_CLK_PIN, PDM_PWR_PIN);
  
  // configure the data receive callback
  PDM.onReceive(onPDMdata);
  
  if (!PDM.begin(1, 16000)) {
    Serial.println("Failed to start PDM!");
    while (1);
  }

  analogReadResolution(12);
  
}


boolean detShot=false;

long mil=millis();

void loop() {

  if(millis()-mil>10000){
//    Serial.println("Sending bat data");
    mil=millis();
    send_bat_status();
  }
  
  if (samplesRead && !detShot) {
    for (int i = 0; i < samplesRead; i++) {
          vReal[i] = (double)sampleBuffer[i]; 
          vImag[i]=0;  
//          j++;

//          if(j>=1024){
//            while(j--){
//              Serial.println(vReal[1024-j]);
//            }
//            j=0;
//          }
    }

    samplesRead = 0;
    
  }
//  if(j>=512){
    detShot=true;
    detectShot();
//    j=0;
    detShot=false; 
//  }
  
}

void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable / 2;
}

void lorawan_has_joined_handler(void)
{
  Serial.println("OTAA Mode, Network Joined!");

  lmh_error_status ret = lmh_class_request(g_CurrentClass);
  if (ret == LMH_SUCCESS)
  {
    delay(1000);
    TimerSetValue(&appTimer, LORAWAN_APP_INTERVAL);
    TimerStart(&appTimer);
  }
}
/**@brief LoRa function for handling OTAA join failed
*/
static void lorawan_join_failed_handler(void)
{
  Serial.println("OTAA join failed!");
  Serial.println("Check your EUI's and Keys's!");
  Serial.println("Check if a Gateway is in range!");
}
/**@brief Function for handling LoRaWan received data from Gateway
 *
 * @param[in] app_data  Pointer to rx data
 */
void lorawan_rx_handler(lmh_app_data_t *app_data)
{
  Serial.printf("LoRa Packet received on port %d, size:%d, rssi:%d, snr:%d, data:%s\n",
          app_data->port, app_data->buffsize, app_data->rssi, app_data->snr, app_data->buffer);
}

void lorawan_confirm_class_handler(DeviceClass_t Class)
{
  Serial.printf("switch to class %c done\n", "ABC"[Class]);
  // Informs the server that switch has occurred ASAP
  m_lora_app_data.buffsize = 0;
  m_lora_app_data.port = gAppPort;
  lmh_send(&m_lora_app_data, g_CurrentConfirm);
}

void send_lora_frame(void)
{
  if (lmh_join_status_get() != LMH_SET)
  {
//    Serial.println("error here");
    //Not joined, try again later
    return;
  }

  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = 'G';
  m_lora_app_data.buffer[i++] = 'u';
  m_lora_app_data.buffer[i++] = 'n';
  m_lora_app_data.buffer[i++] = 's';
  m_lora_app_data.buffer[i++] = 'h';
  m_lora_app_data.buffer[i++] = 'o';
  m_lora_app_data.buffer[i++] = 't';
  m_lora_app_data.buffer[i++] = '!';
  m_lora_app_data.buffsize = i;

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
//  Serial.printf("lmh_error_status error: %d\n", error);
  if (error == LMH_SUCCESS)
  {
    count++;
//    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
}


//static uint8_t mydata[] = { 0,0,0,0,0,0,0,0};

void send_bat_status(void){

//  Serial.println("Sending bat data inside");
  if (lmh_join_status_get() != LMH_SET)
  {
//    Serial.println("error here");
    //Not joined, try again later
    return;
  }

  float bat_val=readVBAT();
  String mydata = String(bat_val);

//  Serial.println(mydata);

  uint32_t i = 0;
  memset(m_lora_app_data.buffer, 0, LORAWAN_APP_DATA_BUFF_SIZE);
  m_lora_app_data.port = gAppPort;
  m_lora_app_data.buffer[i++] = mydata[0];
  m_lora_app_data.buffer[i++] = mydata[1];
  m_lora_app_data.buffer[i++] = mydata[2];
  m_lora_app_data.buffer[i++] = mydata[3];
  m_lora_app_data.buffer[i++] = mydata[4];
  m_lora_app_data.buffer[i++] = mydata[5];
  m_lora_app_data.buffer[i++] = mydata[6];
  m_lora_app_data.buffsize = i;

  lmh_error_status error = lmh_send(&m_lora_app_data, g_CurrentConfirm);
//  Serial.printf("lmh_error_status error: %d\n", error);
  if (error == LMH_SUCCESS)
  {
    count++;
//    Serial.printf("lmh_send ok count %d\n", count);
  }
  else
  {
    count_fail++;
    Serial.printf("lmh_send fail count %d\n", count_fail);
  }
}
