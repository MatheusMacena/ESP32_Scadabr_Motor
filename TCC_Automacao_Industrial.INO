/*
  Autor: Matheus Macena Bastos
  TCC de Pós-graduação em Automação Industrial: SCADA de baixo custo para motores elétricos utilizando o ESP32 e o Scadabr
  Universidade Cândido Mendes - 2021
  Referências:
  http://github.com/andresarmento/modbus-arduino
  http://forum.scadabr.com.br/t/modbus-ip-com-esp32/1678
  https://www.filipeflop.com/blog/medidor-de-energia-eletrica-com-arduino/
  https://www.filipeflop.com/blog/monitorando-temperatura-e-umidade-com-o-sensor-dht11/
  https://circuitdigest.com/microcontroller-projects/mpu6050-gyro-sensor-interfacing-with-esp32-nodemcu-board
  
  
*/
// BIBLIOTECAS

//Modbus
#include <Modbus.h>
#include <ModbusIP_ESP32.h>
ModbusIP mb; //Objeto ModbusIP 

// Sensor de temperatura e umidade DHT11
#include "DHT.h"
#define DHTPIN 27 // GPIO27
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);
long ts;

// Sensor de vibração GY521
#include <Wire.h>
const int MPU_addr=0x68;  // endereço I2C
int16_t AcX; // variável do acelerômetro eixo X

// Sensor de corrente SC013
#include "EmonLib.h" 
EnergyMonitor emon1;
int pino_sct = 34; //GPIO34

//Registradores Modbus
const int MOTOR_COIL = 100; // comando liga desliga
const int SENSOR_T_IREG = 101; // temperatura
const int SENSOR_U_IREG = 102; // umidade
const int STATUS_HREG = 103; // 0=desligado, 1=ligado, 2= n responde, 3= alta temperatura, 4= alta vibração, 5= sobrecorrente, 6= alta umidade
const int RESET_HREG = 104; // 0=set, 1=reset
const int SENSOR_CORR_IREG = 105; // corrente
const int SENSOR_VIB_IREG = 106; // vibração

//Pinos usados
const int MotorPin = 33; // Ligar o motor



void setup() {
  
   Serial.begin(115200); // inicia a comunicação

   mb.config("DM2H_ESCRITORIO", "helo2020");  //Configuração de SSID e PASSWORD
  
   while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("."); //Mostra no monitor o status da conexão e o IP
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  // Configura os pinos usados
  pinMode(MotorPin, OUTPUT);
  pinMode(DHTPIN, INPUT);
  pinMode(pino_sct, INPUT);
  
  // Inicializa o DHT11 
  dht.begin(); 

  // Inicializa o GY521
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // registrador
  Wire.write(0);     // setar em 0 (acorde o sensor)
  Wire.endTransmission(true);
 
  // Inicializa o SCT013
  emon1.current(pino_sct, 7);
  
  // Adiciona registradores Modbus
  mb.addCoil(MOTOR_COIL);
  mb.addIreg(SENSOR_T_IREG);
  mb.addIreg(SENSOR_U_IREG);
  mb.addHreg(STATUS_HREG);
  mb.addHreg(RESET_HREG);
  mb.addIreg(SENSOR_CORR_IREG);
  mb.addIreg(SENSOR_VIB_IREG);
  

  //Inicia o motor desligado
  mb.Coil(MOTOR_COIL, 1); 
  mb.Hreg(STATUS_HREG, 0);
 
}



void loop() {
   //Inicializa a requisicção de dados
   mb.task();

// COMANDO DO MOTOR

   // Comando liga/desliga. Só funciona se não estiver em falha
   if (mb.Hreg(STATUS_HREG)<2){
   mb.task();
   digitalWrite(MotorPin, mb.Coil(MOTOR_COIL)); // liga o motor
   }

 
// SENSORES

  float h = dht.readHumidity(); // variável umidade
  float t = dht.readTemperature(); // variável temperatura
  if (t>50){
    mb.task();
    t=50;
  }
  if (h>100){
    mb.task();
    h=100;
  }
  if (t<0){
    mb.task();
    t=0;
  }
  if (h<0){
    mb.task();
    h=0;
  }
  
  mb.Ireg(SENSOR_T_IREG, t); // salva o valor de temperatura no registrador 101
  mb.Ireg(SENSOR_U_IREG, h); // salva o valor de umidade no registrador 102

  // Sensor de vibração GY521
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // começa com o registro 0x3Bh+
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // requisita 14 registros
  AcX=(Wire.read()<<8|Wire.read())-1800; 
  if(AcX<=0){
    mb.task();
    AcX=0; // ignora valores negativos
  }
  if(AcX>3000){
    mb.task();
    AcX=3000; // ignora valores negativos
  }

    mb.Ireg(SENSOR_VIB_IREG, AcX); //salva o valor de vibração no registrador 106
  
   
   // Sensor de corrente SCT03
   double Irms = emon1.calcIrms(1480); 
   if (Irms<0){
    mb.task();
    Irms=0;
   }
   if (Irms>30){
    mb.task();
    Irms=30;
   }
   mb.Ireg(SENSOR_CORR_IREG, Irms); //salva o valor de corrente no registrador 105
         


  
//CONDIÇÔES DE STATUS   
  // Se o motor for desligado e não estiver em falha
  if ((mb.Coil(MOTOR_COIL)==1) and (mb.Hreg(STATUS_HREG)<2)){
    mb.task();
    mb.Hreg(STATUS_HREG, 0); // motor desligado
  }

  // Se o motor for ligado sem falhas
  if ((mb.Coil(MOTOR_COIL)==0) and (mb.Hreg(STATUS_HREG)<2)){
    mb.task();
    mb.Hreg(STATUS_HREG, 1); // motor ligado
  }
  
// Falhas

    // Falha por falta de corrente
   if ((mb.Hreg(STATUS_HREG)==1) and (mb.Hreg(SENSOR_CORR_IREG)==0.0)){
    mb.task();
    delay(1500);
    mb.Hreg(STATUS_HREG, 2); // falha no comando
    digitalWrite(MotorPin, 1);// desliga o motor
    }
    
    // Falha por temperatura alta
    if ((mb.Coil(MOTOR_COIL)==0) and (mb.Ireg(SENSOR_T_IREG)>=40)){
    mb.task();
    delay(1500);
    mb.Hreg(STATUS_HREG, 3); //alta temperatura
    digitalWrite(MotorPin, 1); // desliga o motor
    }
    
    // Falha por alta vibração
    if ((mb.Coil(MOTOR_COIL)==0) and (mb.Ireg(SENSOR_VIB_IREG)>2000)){
    mb.task();
    delay(1800);
    mb.Hreg(STATUS_HREG, 4); //alta vibração
    digitalWrite(MotorPin, 1);// desliga o motor
    }

    // Falha por corrente altas
    if ((mb.Coil(MOTOR_COIL)==0) and (mb.Ireg(SENSOR_CORR_IREG)>10) ){
    mb.task();
    delay(1800);
    mb.Hreg(STATUS_HREG, 5); //sobrecorrente
    digitalWrite(MotorPin, 1);// desliga o motor
    }

    // Falha por umidade altas
    if ((mb.Coil(MOTOR_COIL)==0) and (mb.Ireg(SENSOR_U_IREG)>=95)){
    mb.task();
    delay(1500);
    mb.Hreg(STATUS_HREG, 6); //alta umidade
    digitalWrite(MotorPin, 1);// desliga o motor
    }

   
  
      
// RESET DE FALHAS

  // Reset quando o motor estiver em falha
  if ((mb.Hreg(RESET_HREG)==1) and mb.Hreg(STATUS_HREG)>=2){
    mb.task();
    mb.Coil(MOTOR_COIL, 1); 
    mb.Hreg(STATUS_HREG, 0);
    mb.Hreg(RESET_HREG, 0);
      }

  // Se não houver falha, não faça nada    
  if (mb.Hreg(RESET_HREG)==1){
    mb.task();
    delay(30);
    mb.Hreg(RESET_HREG, 0);
  }
    
   

  } // ultimo }


  
