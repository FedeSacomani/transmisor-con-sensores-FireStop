/*
  Proyecto Final "Fire Stop", Ing Electronica 2023
  Sacomani Federico, Tealdi Diego
  Transmisor version final
 */

 //Librerias utilizadas
#include "Arduino.h"
#define E32_TTL_1W                  //Es necesario el define antes de incluir la libreria LoRa, para indicar cual es el modulo utilizado
#include "LoRa_E32.h"
//#include "SoftwareSerial.h"
#include "HardwareSerial.h"
#include "DHT.h"
#include "MHZ19.h"
#include "Wire.h"

//Definiciones de pines
//#define D2              16 //o 0           //Rx-Pines M0 y M1 E32
//#define D3              17 //o 1           //Tx-Pines M0 y M1 E32
#define pinDHT          5            
#define anemom          25
#define MQ              15
#define POS             34             
#define Batery          35


//#define PINSleep        GPIO_NUM_33   //Pin utlizado como interrupcion para sleep
#define uS_TO_S_FACTOR  1000000ULL    // Factor de conversion de micro seg a seg 
#define TIME_TO_SLEEP   180          // Tiempo en segundos a dormir la ESP32 
#define T1              26            // Salida a transistor para Temp y Humo
#define T2              27            // Salida a transistor para Lora, anemometro, veleta y humedad
//#define LED             23            //Led de demostracion

LoRa_E32 e32ttl(&Serial2);

DHT dht(pinDHT, DHT22); //Crear variable DHY

//MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO); //Crear variable termocupla

//Declaracion de funcion para imprimira parametros de configuracion de E32
void printParameters(struct Configuration configuration);

//Variables utlizadas en el proyecto
int numero = 0;
int temp = 0;
int humedad = 0;
int velocidad = 0;
int pos = 0;
int monox = 0;
int disp = 2;
int resp = 0;
int tiempo = 0;
int tiempo2 = 0;
float bateria = 0;
int bat=0;
int MQ2=0 ;
volatile int wSCount = 0;

unsigned long lastWindS =0;

unsigned long getDataTimer = 0;

//Variable en memoria para contar reinicios de sleep
RTC_DATA_ATTR int reinicios = 0;

char mensaje [56]= {0}; 

 
void setup()
{
	Serial.begin(9600);
  
	while (!Serial) {
	    ; // Esperar coneccion del puerto serial
    }
  
  //Declaracion de pines de salida
  pinMode(T1, OUTPUT);
  pinMode(T2, OUTPUT);
  
  //Encendido de pines
  
  digitalWrite(T2, HIGH);

	e32ttl.begin(); //Iniciar LoRa E32

	ResponseStructContainer c; //Estructura de parametros del dispositivo
	c = e32ttl.getConfiguration(); //Obtencions de configuracion
	Configuration configuration = *(Configuration*) c.data;
	//Configuraciones disponibles para el E32
	configuration.ADDH = 0x00;  //Direccion High
	configuration.ADDL = 0x00;  //Direccion Low
	configuration.CHAN = 0x17;  //Canal
  
	configuration.OPTION.fixedTransmission = FT_FIXED_TRANSMISSION; //Modo de transmisión
  configuration.OPTION.wirelessWakeupTime = WAKE_UP_250;          //Tiempo de wake up
  configuration.OPTION.fec = FEC_1_ON;                            //Corrección de errores FEC
  configuration.OPTION.transmissionPower = POWER_30;              //Potencia de transmision
  configuration.OPTION.ioDriveMode = IO_D_MODE_PUSH_PULLS_PULL_UPS;//Modo de funcionamiento

  configuration.SPED.airDataRate = AIR_DATA_RATE_001_12;          //Tasa de transmision
  configuration.SPED.uartBaudRate = UART_BPS_9600;                //Velocidad de comunicacion UART
  configuration.SPED.uartParity = MODE_00_8N1;                    //Paridad de la UART
	e32ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE); //Seteo de configuracion anterior
	printParameters(configuration);                                 //Impresion de parametros configurados
	c.close();

  dht.begin();  //Iniciar DHT

  //Indicacion de pines de entrada
  pinMode(anemom, INPUT_PULLUP);
  attachInterrupt(anemom, medirViento, FALLING);
  analogReadResolution(12);
  //pinMode(MQ7, INPUT_PULLUP);
 
  delay(100);
  //Apagamos todos los transistor
  digitalWrite(T2, LOW);
  
  //Aumentamos numero de reinicios e imprimimos
  ++reinicios;
  Serial.println("Reinicio numero: " + String(reinicios));

  //Razon de wake-up
  print_wakeup_reason();

  //Formas de sleep mode de ESP32
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); //Por tiempo
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);//all RTC Peripherals aren´t powered
  //esp_sleep_enable_ext0_wakeup(PINSleep,1);                    //Por interrupción
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");
    
  delay(100);
}

void loop()
{
  
  delay(100);
  
  /*if(reinicios < 5 || reinicios==6 || reinicios==7 || reinicios==8 || reinicios==9 )
  {  
    Serial.println("Durmiendo..... \n");
    esp_deep_sleep_start();    
  } */
  
  digitalWrite(T1, HIGH);
  
  Serial.println("Midiendo, Tem, Hum, vel, direccion,  Bateria y MQ2 \n");
  delay(8000); //Espera de calentamiento de sensor de humo
  //Mediciones
  leerdht();
  Serial.println(temp);
  Serial.println(humedad);
  velocidad= (int) getSpeed();
  Serial.println(velocidad);
  getPos();
  Serial.println(pos);
  getBat();
  Serial.println(bat);  
  getMQ2();
  Serial.println(MQ2);
  
  digitalWrite(T1, LOW);
  Serial.println("Listo \n");
  delay(500);

  //Condiciones de segunda etapa, por cantidad de reinicios o parametros fuera de lugar
 if(reinicios == 2 || MQ2 == 3500 || temp >= 50){
  //Encendido de segundo transistor
  digitalWrite(T2, HIGH);
  Serial.println("envio \n");
    
  //Tiempo utilizado para el envio de datos
  tiempo=millis();
  //Mientras no se reciba una acuse de recibo, o pasen 15 segundos, se seguirá dentro de la función while
  while((resp != 2) && ((millis()-tiempo) <15000)){
    sprintf(mensaje, "%0.1d %0.2d %0.2d %0.3d %0.3d %0.3d %0.4d", disp, temp, humedad, velocidad, pos, bat, MQ2);
    ResponseStatus rs = e32ttl.sendFixedMessage(0x00, 0x00, 0x17, mensaje);
    Serial.println(mensaje);
    Serial.println("Enviado \n");
      //Tiempo entre envio y envio, en caso de no recibir acuse de recibo
      //Es decir, en este caso, se espera un total de 15 segundos o el acuse de recibo, pero si no se recibe un acuse dentro de los 5 segundos, se intenta otra transmision.
      //Por lo tanto, se estarian intentando 3 intentos, 1 cada 5 segundos, de transmision, antes de entrar en modo sleep si no recibe acuse
      tiempo2=millis();
      while((resp != 2) && ((millis()-tiempo2) <5000)){
        if (e32ttl.available()  > 1){
         Serial.println("Esperando respuesta");
         ResponseContainer rs = e32ttl.receiveMessage();
         // First of all get the data
         String message = rs.data;
         message.remove(0,3);
         convertir(message);
         if(resp == 2){
           Serial.println("Resp recibida \n");
           Serial.println(resp);
         }
        }
      }
    }

    resp=0;   
    digitalWrite(T2, LOW);
    Serial.println("Todo apagado \n");
    reinicios=0; 
  }

  //condicion para que no sea infinito los reinicios,por si hay algun error 
  if(reinicios > 2 )
  {
    reinicios=0;
  }
  
  Serial.println("Durmiendo..... \n");
  esp_deep_sleep_start();

}

//Funcion de lectura de DHT
void leerdht(){
 
    float hum = dht.readHumidity();
    float t = dht.readTemperature();
    
    if (isnan(t) || isnan(hum)){
    Serial.println("Falla lectura sensor DHT22");
    delay(2000);
    hum = 1;
    t=1;
    }
 
   humedad=int(hum);
   temp=int(t);
  }
//Funcion de cuenta mediante interrupcion para velocidad del viento
void medirViento() {
  unsigned long current = millis();
  
  if (current - lastWindS >= 10){
    lastWindS = current;
    wSCount++;
    }
  
  }
//Funcion para obtener la velocidad con la cuenta de interrupciones realizadas anteriormente
float getSpeed(){
  
  const float kmH = 2.4;
  float windSpeed = (float)wSCount * kmH;
  wSCount=0;

  return windSpeed/2;
  }
//Medicion de posicion del viento, mediante divisores resistivos, por adc
void getPos()
{
  int adcValor=0;
  for(int i=0; i<30; i++)
    adcValor+=analogRead(POS);
  adcValor=adcValor/30;
  
 
  if(3000<adcValor && adcValor<3200)//3110 NORTE
    pos= 360; //NORTE
  if(1300<adcValor && adcValor<1750)//1645
    pos= 22; //
  
  if(1750<adcValor && adcValor<2100)//1870
    pos= 45; //        

  if(200<adcValor && adcValor<260)//240
    pos= 67; //
    
  if(260<adcValor && adcValor<350)//280 ESTE
    pos= 90; //ESTE
    
  if(50<adcValor && adcValor<200)//155
    pos= 112; //        
  
  if(500<adcValor && adcValor<800)//695
    pos= 135; //
  
  if(350<adcValor && adcValor<500)//430
    pos= 157; //
  
  if(1050<adcValor && adcValor<1300)//1150 SUR
    pos= 180; // SUR       
  
  if(800<adcValor && adcValor<1050)//960
    pos= 202; //
  
  if(2450<adcValor && adcValor<2600)//2500
    pos= 225; //
  
  if(2100<adcValor && adcValor<2450)//2400
    pos= 247; //
  
  if(3800<adcValor && adcValor<4095)//3900 OESTE
    pos= 270; //OESTE
  
  if(3000<adcValor && adcValor<3400)//3280
    pos= 292; //        
  
  if(3400<adcValor && adcValor<3800)//3570
    pos= 315; //
  
  if(2600<adcValor && adcValor<3000)//2790
    pos= 337; //
 }

//Obtencion de voltaje de bateria mediante ADC
void getBat(){

  int adcValor=0;

  for(int i=0; i<50; i++)
    adcValor+=analogRead(Batery);

  adcValor=adcValor/50; 
  
  bateria=(adcValor*3.3)/(4095);
  bat=bateria*100; 
  }

//Medicion de monoxido de carbono o humo
void getMQ2(){
  //Mediciones en PPM, no pueden ser contrastadas, por eso se hace medicion digital
  for(int i=0; i<50; i++)
    MQ2 += analogRead(MQ);
  
  MQ2 = MQ2/50; 
  //Serial.println(MQ2); 
  //delay(1000);
  
  }

//Funcion de conversion de respuesta recibida
void convertir (String message){
  resp=(String(message[0]).toInt());
  Serial.println(resp);
  }

//Funcion para imprimir la razon de wake-up
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason){
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup causado por interrupcion externa usando RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup causado por interrupcion externa usando RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup causado por tiempo"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup causado por touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup causado por programa ULP"); break;
    default : Serial.printf("Wakeup no fue causado por deepSleep: %d\n",wakeup_reason); break;
    }
  }
//Funcion para imprimir los parametros de dispositivo LoRa
void printParameters(struct Configuration configuration) {
	Serial.println("----------------------------------------");

	Serial.print(F("HEAD : "));  Serial.print(configuration.HEAD, BIN);Serial.print(" ");Serial.print(configuration.HEAD, DEC);Serial.print(" ");Serial.println(configuration.HEAD, HEX);
	Serial.println(F(" "));
	Serial.print(F("AddH : "));  Serial.println(configuration.ADDH, BIN);
	Serial.print(F("AddL : "));  Serial.println(configuration.ADDL, BIN);
	Serial.print(F("Chan : "));  Serial.print(configuration.CHAN, DEC); Serial.print(" -> "); Serial.println(configuration.getChannelDescription());
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit     : "));  Serial.print(configuration.SPED.uartParity, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTParityDescription());
	Serial.print(F("SpeedUARTDatte  : "));  Serial.print(configuration.SPED.uartBaudRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getUARTBaudRate());
	Serial.print(F("SpeedAirDataRate   : "));  Serial.print(configuration.SPED.airDataRate, BIN);Serial.print(" -> "); Serial.println(configuration.SPED.getAirDataRate());

	Serial.print(F("OptionTrans        : "));  Serial.print(configuration.OPTION.fixedTransmission, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFixedTransmissionDescription());
	Serial.print(F("OptionPullup       : "));  Serial.print(configuration.OPTION.ioDriveMode, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getIODroveModeDescription());
	Serial.print(F("OptionWakeup       : "));  Serial.print(configuration.OPTION.wirelessWakeupTime, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getWirelessWakeUPTimeDescription());
	Serial.print(F("OptionFEC          : "));  Serial.print(configuration.OPTION.fec, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getFECDescription());
	Serial.print(F("OptionPower        : "));  Serial.print(configuration.OPTION.transmissionPower, BIN);Serial.print(" -> "); Serial.println(configuration.OPTION.getTransmissionPowerDescription());

	Serial.println("----------------------------------------");

  }
