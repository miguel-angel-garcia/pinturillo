/*
 * Método de calibración:
 * 1.- Después de iniciar, espera 1 segundo
 * 2.- El led parpadea 2 veces (0,5 segundos cada parpadeo)
 * 3.- Espera 1 segundo y calibra a BLANCO
 * 4.- El led parpadea 3 veces (0,5 segundos cada parpadeo)
 * 5.- Espera 1 segundo y calibra a NEGRO
 * 6.- El led parpacea 5 veces y finaliza la calibración
 * 7.- Espera 3 segundos 
 */

#include <Servo.h>

// Creamos los objetos para los servos
Servo ruedaIzq;
Servo ruedaDer;
Servo pincel;
Servo brazos;

// Estados del robot
#define DRIVE_FORWARD 0
#define TURN_SOFT_LEFT 1
#define TURN_LEFT 2
#define TURN_SOFT_RIGHT 3
#define TURN_RIGHT 4
// Colores detectados
#define NEGRO 0
#define GRIS 1
#define BLANCO 2
// Parámetros servos
#define MAX_VEL_IZQ 96 // 180 -> Máx (funciona bien con 96 y 106)
#define MIN_VEL_IZQ 0
#define MAX_VEL_DER 71 // 0 -> Máx (funciona bien con 71 y 61)
#define MIN_VEL_DER 180
#define PARADO_IZQ 86
#define PARADO_DER 81
#define MAX_PINCEL_IZQ 114
#define MAX_PINCEL_DER 82
#define MAX_BRAZOS_IZQ 135
#define MAX_BRAZOS_DER 45
// Parámetros PID
#define Kp 1 // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
#define Kd 40// experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 


// Constantes
const int serialPeriod = 500; // Sólo se imprime en la consola cada 1/2 segundo
const int loopPeriod = 20; // Período de 20 ms = frecuencia de 50 Hz
const int pincelPeriod = 500; // Sólo se mueve el pincel cada 1/2 segundo
const int LDRRight = A0; // Pin del LDR derecho
const int LDRLeft = A1;  // Pin del LDR izquierdo
const int buzzer = 12;   // Pin del buzzer
const int pinPincel = 2;
const int pinBrazos = 3; // Los servos llevan un cable en Y y sólo necesitan un canal
const int pinRuedaIzq = 10;
const int pinRuedaDer = 9;

// Variables globales
int valLDRDer;
int valLDRIzq;
float medBlancoDer = 0;
float medBlancoIzq = 0;
unsigned long timeSerialDelay = 0;
unsigned long timeLoopDelay = 0;
unsigned long timePincelDelay = 0;

int gradoGiro = 90;
int gradoPincel = 93; // Centrado a 93
int gradoBrazos = 90;
long resMinRight = 0;
long resMaxRight = 0;
long resMinLeft = 0;
long resMaxLeft = 0;
long tmpMin = 0;
long umbralRight;
long umbralLeft;

// Variables de estado
int state = DRIVE_FORWARD; // 0 = hacia adelante (por defecto), 1 = girar a la izquierda, 2 = girar a la derecha...
int lastState = DRIVE_FORWARD;
int stateLDRDer = BLANCO;
int stateLDRIzq = BLANCO;
int error = 0;
int lastError = 0;
int motorSpeed;

void blink_led_buzz(int times)
{
  int j;
  for(j = 0; j < times; j++)
  {
      digitalWrite(LED_BUILTIN, HIGH);
      tone(buzzer, 2610);
      delay(500); // Espera medio segundo
      digitalWrite(LED_BUILTIN, LOW);
      noTone(buzzer);
      delay(500);
  }
}

void buzz_left()
{
  tone(buzzer, 523, 100); // do
}

void buzz_right()
{
  tone(buzzer, 657, 100); // mi
}

void buzz_forward()
{
  noTone(buzzer);
}

void setup() 
{
  int i;
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);

  // Inicialización de los servos
  ruedaIzq.attach(pinRuedaIzq);
  ruedaDer.attach(pinRuedaDer);
  pincel.attach(pinPincel);
  brazos.attach(pinBrazos);
  ruedaIzq.write(PARADO_IZQ);
  ruedaDer.write(PARADO_DER);
  pincel.write(gradoPincel);
  brazos.write(gradoBrazos);
  
  // Calibración de los sensores LDR
  delay(2000); // Espera 2 segundos
  blink_led_buzz(2); // El led parpadea dos veces y el buzzer también
  delay(1000); // Espera 1 segundo
  
  // Primero el blanco
  for(i = 0; i < 1000; i++)
  {
    resMaxRight = resMaxRight + analogRead(LDRRight);
    resMaxLeft = resMaxLeft + analogRead(LDRLeft);
    delay(2);
  }
  resMaxRight = resMaxRight/1000;
  resMaxLeft = resMaxLeft/1000;
  Serial.print("Blanco Derecho: ");
  Serial.println(resMaxRight);
  Serial.print("Blanco Izquierdo: ");
  Serial.println(resMaxLeft);

  blink_led_buzz(3); // El led parpadea tres veces y el buzzer también
  delay(1000); // Espera 1 segundo
  // Después para el negro
  for(i = 0; i < 1000; i++)
  {
    resMinRight = resMinRight + analogRead(LDRRight);
    resMinLeft = resMinLeft + analogRead(LDRLeft);
    delay(2);
  }
  resMinRight = resMinRight/1000;
  resMinLeft = resMinLeft/1000;
  Serial.print("Negro Derecho: ");
  Serial.println(resMinRight);
  Serial.print("Negro Izquierdo: ");
  Serial.println(resMinLeft);

  // Si se midió al revés, se intercambian
  if(resMinRight > resMaxRight)
  {
    tmpMin = resMinRight;
    resMinRight = resMaxRight;
    resMinRight = tmpMin;
  }
  if(resMinLeft > resMaxLeft)
  {
    tmpMin = resMinLeft;
    resMinLeft = resMaxLeft;
    resMinLeft = tmpMin;
  }

  // Asignamos el umbral
  umbralRight = (resMinRight + resMaxRight) / 2;
  umbralLeft = (resMinLeft + resMaxLeft) / 2;
  Serial.print("Umbral Derecho: ");
  Serial.println(umbralRight);
  Serial.print("Umbral Izquierdo: ");
  Serial.println(umbralLeft);
  // Señal de finalización de la calibración
  blink_led_buzz(5); // El led parpadea cinco veces y el buzzer también
  delay(1000);

}

/**** Bucle principal ****/
void loop()
{
  // Salida de depuración
  debugOutput();
  // Control del sensor de ultrasonidos
  if(millis() - timeLoopDelay >= loopPeriod)
  {
    readLDRs();
    stateMachine();
    moveMachine();
    movePincelyBrazos();
    timeLoopDelay = millis();
  }

}

/**** Procedimientos ****/

// Salida de debug por el puerto serie
void debugOutput()
{
  if((millis() - timeSerialDelay) > serialPeriod)
  {
    // Lectura del LDR
    Serial.print("Lectura Der: ");
    Serial.println(valLDRDer);
    if(stateLDRDer == NEGRO)
      Serial.println("Derecha: NEGRO");
    else
      if(stateLDRDer == GRIS)
        Serial.println("Derecha: GRIS");
      else
        Serial.println("Derecha: BLANCO");

    Serial.print("Lectura Izq: ");
    Serial.println(valLDRIzq);
    if(stateLDRIzq == NEGRO)
      Serial.println("Izquierda: NEGRO");
    else
      if(stateLDRIzq == GRIS)
        Serial.println("Derecha: GRIS");
      else
        Serial.println("Izquierda: BLANCO");
    // Estado
    Serial.print("Estado: ");
    if(state == TURN_LEFT)
      Serial.println("TURN_LEFT");
    else
      if(state == TURN_SOFT_LEFT)
        Serial.println("TURN_SOFT_LEFT");
      else
        if(state == TURN_RIGHT)
          Serial.println("TURN_RIGHT");
        else
          if(state == TURN_SOFT_RIGHT)
            Serial.println("TURN_SOFT_RIGHT");
          else
            Serial.println("DRIVE_FORWARD");
    timeSerialDelay = millis();
  }
}

// Lectura del sensor de luz izquierdo
long readLeftLDR()
{
  long lectLeft = analogRead(LDRLeft);
  // Normalizamos
  lectLeft = 100*(lectLeft - resMinLeft)/(resMaxLeft - resMinLeft);
  if(lectLeft < 0) 
    lectLeft = 0;
  return(lectLeft);
}

// Lectura del sensor de luz derecho
long readRightLDR()
{
  long lectRight = analogRead(LDRRight);
  // Normalizamos
  lectRight = 100*(lectRight - resMinRight)/(resMaxRight - resMinRight);
  if(lectRight < 0)
    lectRight = 0;
  return(lectRight);
}

// Lectura de los sensores de luz y establecimiento del estado de los mismos
// Contemplamos tres estados: NEGRO, GRIS y BLANCO
void readLDRs()
{
  // Lectura de los datos de los sensores LDR
  valLDRDer = readRightLDR();
  valLDRIzq = readLeftLDR();
  if(valLDRDer < 33)
    stateLDRDer = NEGRO;
  else
    if(valLDRDer < 66)
      stateLDRDer = GRIS;
    else
      stateLDRDer = BLANCO;
  if(valLDRIzq < 33)
    stateLDRIzq = NEGRO;
  else
    if(valLDRIzq < 66)
      stateLDRIzq = GRIS;
    else
      stateLDRIzq = BLANCO;
}

// En función de los estados de los sensores se determina el estado de los motores
void stateMachine()
{
  lastState = state;
  lastError = error;
  if(stateLDRIzq == NEGRO && stateLDRDer == NEGRO)
  {
    state = DRIVE_FORWARD; // Seguir adelante
    error = 0;
  }
  if(stateLDRIzq == NEGRO && stateLDRDer == GRIS)
  {
    state = TURN_SOFT_LEFT; // Si es gris el derecho, vamos a la izquierda
    error++;
  }
  if(stateLDRIzq == NEGRO && stateLDRDer == BLANCO)
  {
    state = TURN_LEFT; // Si es blanco el derecho será más a la izquierda
    error += 2;
  }
  if(stateLDRIzq == GRIS && stateLDRDer == NEGRO)
  {
    state = TURN_SOFT_RIGHT; // Si es gris el izquierdo, vamos a la derecha
    error++;
  }
  if(stateLDRIzq == GRIS && stateLDRDer == GRIS)
  {
    state = DRIVE_FORWARD; // Seguir adelante
    error = 0;
  }
  if(stateLDRIzq == GRIS && stateLDRDer == BLANCO)
  {
    state = TURN_LEFT; // Si es blanco el derecho será más a la izquierda
    error += 3;
  }
  if(stateLDRIzq == BLANCO && stateLDRDer == NEGRO)
  {
    state = TURN_RIGHT; // Si es blanco el izquierdo será más a la derecha
    error += 2;
  }
  if(stateLDRIzq == BLANCO && stateLDRDer == GRIS)
  {
    state = TURN_RIGHT; // Si es gris el derecho, vamos a la derecha
    error += 3;
  }
  if(stateLDRIzq == BLANCO && stateLDRDer == BLANCO)
  {
    state = lastState; // Nos hemos perdido, mantenemos el estado anterior
    error += 4;
  }
}

void moveMachine()
{  
  if(state == DRIVE_FORWARD) // Sigue en una recta
  {
    // Seguir adelante
    buzz_forward();
    ruedaIzq.write(MAX_VEL_IZQ);
    ruedaDer.write(MAX_VEL_DER);
  }
  else
  {
    int rightMotorSpeed = 0;
    int leftMotorSpeed = 0;
    
    // Calculamos la velocidad de giro ajustando el PID
    motorSpeed = Kp * error + Kd * (error - lastError);
    
    if(state == TURN_SOFT_LEFT || state == TURN_LEFT) // Comenzamos una curva -> hay que girar a la izquierda
    {
      // Girar a la izquierda
      buzz_left();
      rightMotorSpeed = MAX_VEL_DER - motorSpeed;
      leftMotorSpeed = MAX_VEL_IZQ - motorSpeed;
    }
    
    if(state == TURN_SOFT_RIGHT || state == TURN_RIGHT) // Comenzamos en una curva -> hay que girar a la derecha
    {
      // Girar a la derecha
      buzz_right();
      rightMotorSpeed = MAX_VEL_DER + motorSpeed;
      leftMotorSpeed = MAX_VEL_IZQ + motorSpeed;
    }
    // Comprobamos que no sobrepasemos los valores máximos y mínimos de velocidad
    if (leftMotorSpeed > MAX_VEL_IZQ ) 
      leftMotorSpeed = MAX_VEL_IZQ;
    if (rightMotorSpeed < MAX_VEL_DER) 
      rightMotorSpeed = MAX_VEL_DER;
    if (leftMotorSpeed < MIN_VEL_IZQ)
      leftMotorSpeed = MIN_VEL_IZQ;
    if (rightMotorSpeed > MIN_VEL_DER)
      rightMotorSpeed = MIN_VEL_DER;    
    
    // Hacemos girar los motores
    ruedaDer.write(rightMotorSpeed);
    ruedaIzq.write(leftMotorSpeed);
  }
}

void movePincelyBrazos()
{
  if(millis() - timePincelDelay >= pincelPeriod)
  {
    // Movimiento del Pincel
    if(gradoPincel >= 90)
      gradoPincel = MAX_PINCEL_DER;
    else
      gradoPincel = MAX_PINCEL_IZQ;
    pincel.write(gradoPincel);
    // Movimiento de los Brazos
    if(gradoBrazos >= 90)
      gradoBrazos = MAX_BRAZOS_DER;
    else
      gradoBrazos = MAX_BRAZOS_IZQ;
    brazos.write(gradoBrazos);
    timePincelDelay = millis();
  }
}

