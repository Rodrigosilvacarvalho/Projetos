
#include <Wire.h>                      
#include "BMP085.h"                   
#include <Tone.h>                      
#include<stdlib.h>                     

/////////////////////////////////////////
/////////////////////////////////////////   variaveis de configuraçao do sistema
short speaker_pin1 = 8;                  // autofalante canal  -
short speaker_pin2 = 9;                  // autofalante canal  +
float vario_climb_rate_start = 0.4;      // (ex. 0,4 inicia a   0.4m/s)
float vario_sink_rate_start = -1.0;      // (ex. 1,1 inicia a  -1.1m/s)
#define SAMPLES_ARR 25                   // define o filtro da sensibilidade do variometro 
#define NMEA_OUT_per_SEC 3               // tempo de saida protocolo NMEA Serial
#define VOLUME 2                         // volume configuração  0-sem som  , 1- volume baixo , 2-volume alto
const float p0 = 101325;                 // Pressao nivel do mar (Pa)
long     Pressure = 101325;              //
float    Altitude=0;                     //
/////////////////////////////////////////
/////////////////////////////////////////


BMP085   bmp085 = BMP085();              
Tone     tone_out1;
Tone     tone_out2;
long     Temperature = 0;

unsigned long get_time1 = millis();
unsigned long get_time2 = millis();
unsigned long get_time3 = millis();

int      my_temperature = 1;
char     altitude_arr[6];             
char     vario_arr[6];               
int      samples=40;
int      maxsamples=50;
float    alt[51];
float    tim[51];
float    beep;
float    Beep_period;

//---------------------------------------------------------------------------------------------------------

static long k[SAMPLES_ARR];
static long Averaging_Filter(long input);
static long Averaging_Filter(long input) 
{
  long sum = 0;
  for (int i = 0; i < SAMPLES_ARR; i++) {
    k[i] = k[i+1];
  }
  k[SAMPLES_ARR - 1] = input;
  for (int i = 0; i < SAMPLES_ARR; i++) {
    sum += k[i];
  }
  return ( sum / SAMPLES_ARR ) ;
}


//---------------------------------------------------------------------------------------------------------

void play_welcome_beep()                 //som inicial 
{
  
     
  for (int aa=300;aa<=1500;aa=aa+100)
  {
    tone_out1.play(aa,200);           
   if (VOLUME==2){ tone_out2.play(aa+5,200);}             
    delay(100);
  }
  for (int aa=1500;aa>=100;aa=aa-100)
  {
    tone_out1.play(aa,200);             
   if (VOLUME==2){ tone_out2.play(aa+5,200);}             
    delay(100);
  }
}



//---------------------------------------------  void setup  ------------------------------------------------------------

void setup()                
{
  Wire.begin();                   
  
//---------------------------------------------------------------------------------------------------------
  
  if (VOLUME==1){
  tone_out1.begin(speaker_pin1);       
  pinMode(speaker_pin2, OUTPUT);      
  digitalWrite(speaker_pin2, LOW); 
  } 
  else if (VOLUME==2){
  tone_out1.begin(speaker_pin1);       
  tone_out2.begin(speaker_pin2);       
  }
  
//---------------------------------------------------------------------------------------------------------
  
  
  bmp085.init(MODE_ULTRA_HIGHRES, p0, false); 
                         
  
//---------------------------------------- -----------------------------------------------------------------
  
  play_welcome_beep();      

//---------------------------------------------------------------------------------------------------------

}
float nmea_vario_cms =0;
float nmea_time_s=0;
float nmea_alt_m=0;
float nmea_old_alt_m=0;


char variostring[6], altstring[6]; 

//--------------------------------------------  void loop  -------------------------------------------------------------

void loop(void)
{
  float time=millis();             
  float vario=0;
  float N1=0;
  float N2=0;
  float N3=0;
  float D1=0;
  float D2=0;
 

//-------------------------------------------- Calculo Pressao ------------------------------------------------------------- 
  bmp085.calcTruePressure(&Pressure);                                  
  long average_pressure = Averaging_Filter(Pressure);                   
  Altitude = (float)44330 * (1 - pow(((float)Pressure/p0), 0.190295));  

//--------------------------------------------- Calculo Altitude ------------------------------------------------------------
 nmea_alt_m = (float)44330 * (1 - pow(((float)average_pressure/p0), 0.190295));
 if ((millis() >= (nmea_time_s+1000))){
 nmea_vario_cms = ((nmea_alt_m-nmea_old_alt_m))*100; 
 nmea_old_alt_m = nmea_alt_m;
 nmea_time_s = millis();
 }

//----------------------------------------------  Variometro -----------------------------------------------------------

  for(int cc=1;cc<=maxsamples;cc++){                                    
    alt[(cc-1)]=alt[cc];                                                
    tim[(cc-1)]=tim[cc];                                               
  };  
  
 
  //now we have altitude-time tables
  alt[maxsamples]=Altitude;                                             
  tim[maxsamples]=time;                                                 
  float stime=tim[maxsamples-samples];
  for(int cc=(maxsamples-samples);cc<maxsamples;cc++){
    N1+=(tim[cc]-stime)*alt[cc];
    N2+=(tim[cc]-stime);
    N3+=(alt[cc]);
    D1+=(tim[cc]-stime)*(tim[cc]-stime);
    D2+=(tim[cc]-stime);
  };

 vario=1000*((samples*N1)-N2*N3)/(samples*D1-D2*D2);
 if ((time-beep)>Beep_period)                         
  {
    beep=time;
    if (vario>vario_climb_rate_start && vario<=10 )
    {
      switch (VOLUME) 
      {
        case 0: 
          break;
        case 1:
          Beep_period=550-(vario*(30+vario));
          tone_out1.play((1400+(200*vario)),420-(vario*(20+vario)));
        case 2:
          Beep_period=550-(vario*(30+vario));
          tone_out1.play((1400+(200*vario)),420-(vario*(20+vario))); 
          tone_out2.play((1406+(200*vario)),420-(vario*(20+vario)));
      }               
    } else if (vario >10) 
    {
     
    switch (VOLUME) 
      {
        case 0: 
          break;
        case 1:
          Beep_period=160;
          tone_out1.play(3450,120);                         
        case 2:
          Beep_period=160;
          tone_out1.play(3450,120);                          
          tone_out2.play(3456,120);
      }               
    } else if (vario< vario_sink_rate_start){           
     
     switch (VOLUME) 
      {
        case 0: 
          break;
        case 1:
          Beep_period=200;
          tone_out1.play(300,340);
        case 2:
          Beep_period=200;
          tone_out1.play(300,340);
          tone_out2.play(320,340);
      }               
    }
  }

}
//The End
