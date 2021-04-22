//PRUEBA
#include <ModbusRtu.h>
#include <EEPROM.h>
//==================DEFINICIONES=====================================//
#define PROGRAMACION 8
#define JUMPER1  13   
#define JUMPER2  12 
#define JUMPER3  11
#define MD0  2 
#define MD1  3
#define EE_DIRECCION_MF_RTU 0
#define ON    0
#define OFF   1
#define LED_AUX 9
#define ID 3
#define RELE1 4
#define RELE2 5
#define RELE3 6
#define RELE4 7
#define IN1 A2
#define IN2 A3
int pruebaffff;
//==================TRAMAS CONFIGURACION DE FRECUENCIA RF================//
const byte CANAL14_430MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X14, 0X40}; 
const byte CANAL15_431MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X15, 0X40};  
const byte CANAL16_432MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X16, 0X40}; 
const byte CANAL17_433MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X17, 0X40}; 
const byte CANAL18_434MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X18, 0X40}; 
const byte CANAL19_435MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X19, 0X40}; 
const byte CANAL20_436MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X1A, 0X40}; 
const byte CANAL21_437MHZ[]  = {0XC0, 0X00, 0X00, 0X1A, 0X1B, 0X40}; 
const size_t LONGITUD_TRAMA_MODULO = sizeof(CANAL14_430MHZ) / sizeof(CANAL14_430MHZ[0]);

//==================DECLARACION DE VARIABLES=====================================//
bool bG_PrimeraEntrada;  
bool rele1, rele2, rele3, rele4; 
unsigned char cG_Recibir[10];
unsigned char DIRECCION_MF_RTU;
int8_t state = 0;
unsigned long tempus;
uint16_t au16data[15]; //La tabla de registros que se desea compartir por la red

//==================INSTANCIA MODBUS=====================================//
Modbus slave(ID, 0, 0); //ID del nodo. 0 para el master, 1-247 para esclavo  


//==================SETUP=====================================//
void setup() 
{
   
  pinMode(PROGRAMACION, INPUT_PULLUP); //PROGRAMACION
  pinMode(A2, INPUT_PULLUP); //ENTRADA IN1
  pinMode(A3, INPUT_PULLUP); //ENTRADA IN2
  pinMode(JUMPER1, INPUT_PULLUP );  //PIN CAMBIO DE FRECUENCIA
  pinMode(JUMPER2, INPUT_PULLUP );  //PIN CAMBIO DE FRECUENCIA
  pinMode(JUMPER3, INPUT_PULLUP );  //PIN CAMBIO DE FRECUENCIA

  
  pinMode(4, OUTPUT);//SALIDA 1
  pinMode(5, OUTPUT);//SALIDA 2
  pinMode(6, OUTPUT);//SALIDA 3
  pinMode(7, OUTPUT);//SALIDA 4
  pinMode(9, OUTPUT); //LED AUXILIAR
  pinMode(2, OUTPUT); //CONTROL MD0
  pinMode(3, OUTPUT); //CONTROL MD1
  pinMode(9, OUTPUT); //LED AUXILIAR
  
  Serial.begin(9600);
  digitalWrite(RELE1, LOW );
  digitalWrite(RELE2, LOW );
  digitalWrite(RELE3, LOW );
  digitalWrite(RELE4, LOW );
  digitalWrite ( MD0, HIGH); //MODO CONFIGURACION
  digitalWrite ( MD1, HIGH); //MODO CONFIGURACION
  delay(2000);
  CONFIG_MODULO(); //CONFIGURAR MODULO RF
  delay(2000);
  digitalWrite ( MD0, LOW); //MODO  OPERATIVO
  digitalWrite ( MD1, LOW); //MODO  OPERATIVO
  delay(500);
  
  slave.begin(9600); //Abre la comunicación como esclavo
  tempus = millis() + 100; //Guarda el tiempo actual + 100ms
  digitalWrite(LED_AUX, HIGH );// PRENDE LED AUXILIAR
  bG_PrimeraEntrada = true; //PRIMERA ENTRADA PARA QUE LEA LOS VALORES EEPROM
  InicializarVariables(); //INICIALIZA LAS VARIABLES

  
  
}

//==================PROGRAMA PRINCIPAL=====================================//

void loop() 

{

 if(digitalRead(PROGRAMACION) == OFF) // SI MODO PROGRAMACION ESTA DESACTIVADO ENTRA
  {
   if(bG_PrimeraEntrada == true)  // PRIMERA ENTRADA PARA INICIALIZAR VARIABLES Y DAR DIRECCION MODBUS
   
    { 
      bG_PrimeraEntrada = false;
      InicializarVariables();
      slave.setID(DIRECCION_MF_RTU);
    }
  
  state = slave.poll( au16data, 15 ); //TABLA DE REGISTROS MODBUS COMPARTIDOS
  
  //Devuelve 0 si no hay pedido de datos
  //Devuelve 1 al 4 si hubo error de comunicación
  //Devuelve mas de 4 si se procesó correctamente el pedido

  if (state > 4) 
  { //Si es mayor a 4 = el pedido fué correcto
    tempus = millis() + 50; //Tiempo actual + 50ms
    digitalWrite(LED_AUX, HIGH);//Prende el led
  }
  if (millis() > tempus) digitalWrite(LED_AUX, LOW );//Apaga el led 50ms después

  //Actualiza los pines de Arduino con la tabla de Modbus
 
   io_poll();
}
    else
  {
   Programacion();
   bG_PrimeraEntrada = true;
   digitalWrite(LED_AUX, LOW );
  } 

}
void io_poll() 
{

if ( digitalRead (IN1) == 1)
       { 
         au16data[1]=0;
       }
       else{ 
       au16data[1]=1;
       }
if ( digitalRead (IN2) == 1)
       { 
         au16data[2]=0;
       }
       else{ 
       au16data[2]=1;
       }
if ( au16data[3]== 1)
       { 
        digitalWrite( RELE1, HIGH);
       }
       else{ 
       digitalWrite( RELE1, LOW);
       }
if ( au16data[4]==1)
       { 
        digitalWrite( RELE2, HIGH);
       }      
        else {
        digitalWrite( RELE2, LOW);
       }
if ( au16data[5]== 1)
       { 
        digitalWrite( RELE3, HIGH);
       }       
        else {
        digitalWrite( RELE3, LOW);
       }
if ( au16data[6]== 1)
       { 
        digitalWrite( RELE4, HIGH);
       }
       else {        
        digitalWrite( RELE4, LOW);
       }

  au16data[7] = analogRead( A1 ); //El valor analógico leido en el pin A0 se guarda en au16data[7]. (siendo 0=0v y 1023=5v)
  au16data[8] = analogRead( A0 );
  au16data[9] = analogRead( A7 ); 
  au16data[10]= analogRead( A6 );
  au16data[11]= DIRECCION_MF_RTU;
  au16data[12] = slave.getInCnt();  //Devuelve cuantos mensajes se recibieron
  au16data[13] = slave.getOutCnt(); //Devuelve cuantos mensajes se transmitieron
  au16data[14] = slave.getErrCnt(); //Devuelve cuantos errores hubieron
    
}


void CONFIG_MODULO()
{
   
  unsigned char ValorJumpers;
  ValorJumpers = 0;
  ValorJumpers = digitalRead(JUMPER1)*4+digitalRead(JUMPER2)*2+digitalRead(JUMPER3);
   digitalWrite(9, LOW );
  switch(ValorJumpers)
  {
    case 0:
    {
     Serial.write(CANAL14_430MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 1:
    {
      Serial.write(CANAL15_431MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 2:
    {
     Serial.write(CANAL16_432MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 3:
    {
     Serial.write(CANAL17_433MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 4:
    {
      Serial.write(CANAL21_437MHZ, LONGITUD_TRAMA_MODULO);
     break; 
    }
    case 5:
    {
      Serial.write(CANAL19_435MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 6:
    {
      Serial.write(CANAL20_436MHZ, LONGITUD_TRAMA_MODULO);
     break;
    }
    case 7:
    {
     Serial.write(CANAL18_434MHZ, LONGITUD_TRAMA_MODULO);
     break;
    } 
  }
    
  return;  
}


void w_eeprom(unsigned char DirEE, unsigned char *pDato, unsigned char cL_CantidadDeBytes)
{
  static unsigned char i;
  for(i=0; i<cL_CantidadDeBytes;i++)
  {
    EEPROM.update(DirEE+i, *(pDato+i));
  }
  return;
}
void r_eeprom(unsigned char *pDato, unsigned char DirEE, unsigned char cL_CantidadDeBytes)
{
  static unsigned char i;
  for(i=0; i<cL_CantidadDeBytes;i++)
  {
    *(pDato+i)=EEPROM.read(DirEE+i);
  }
  return;
}

void InicializarVariables(void)
{ 
 r_eeprom(&DIRECCION_MF_RTU, EE_DIRECCION_MF_RTU, 1);
 
 return;
}

void Programacion(void){
  while(digitalRead(PROGRAMACION) == ON)
  {  digitalWrite(LED_AUX, HIGH);
    if (Serial.available()>0)
      {  
        Serial.readBytes(cG_Recibir,64); //devuelve cantidad de caracteres
        switch (cG_Recibir[0])
         { 
        case 'A':
         {
         if (cG_Recibir[1]> 0 && cG_Recibir[1]<255) 
         {
         DIRECCION_MF_RTU = cG_Recibir[1]; 
         w_eeprom(EE_DIRECCION_MF_RTU,&DIRECCION_MF_RTU , 1);
         Serial.println ("OK");
         }
         else { Serial.println ("Direccion Fuera de Rango");
         }
        break;  
        } 
      }   
    }   
  }
 return;
}
