#include "./src/SD.h"
#include "./src/Servo.h"
#include "./src/sbus.h"
//#include <RC_Receiver.h>

/*ERRORS :
LED --- Message
1   --- Good !
2   --- 
3   --- Problème d'init de la SD Card
4   --- Problème d'ouverture du fichier sur la SD
*/


/* SBUS object, reading SBUS */
bfs::SbusRx sbus_rx_Futaba(&Serial7);
/* SBUS object, writing SBUS */
//bfs::SbusTx sbus_tx(&Serial2);

bfs::SbusRx sbus_rx_ProTronik(&Serial8);

/* SBUS data */
bfs::SbusData dataFutaba;
bfs::SbusData dataProTronik;




#define PWM_FREQ 50.0f   // Fréquence des ESC (50 Hz)
#define PWM_MIN 1000.0f  // Signal minimum (µs) → ESC au repos
#define PWM_MAX 2000.0f  // Signal maximum (µs) → Plein gaz
#define MAX_SBUS 2048.0f
#define MIN_SBUS 0.0f


#define pinBoutonVert 38
#define pinBoutonRouge 39

#define ledPinTeensy 13
#define ledPinVerte 40
#define ledPinRouge 41

#define ventouse1Mag1 14
//#define ventouse1Mag2 
#define ventouse2Mag1 15
//#define ventouse2Mag2 11

#define pinPompeBallastTangage1 10
#define pinPompeBallastTangage2 11
#define pinPompeBallastRoulis1 12
#define pinPompeBallastRoulis2 13


#define pinServoNacelle1 22
#define pinServoNacelle2 23
Servo servoNacelle1;
Servo servoNacelle2;

#define pinServoEchangeBallast1 24
#define pinServoEchangeBallast2 25
Servo servoEchangeBallast1;
Servo servoEchangeBallast2;


Servo moteurs[8];
const unsigned int pin_moteurs[8] = { 2, 3, 4, 5, 6, 7, 8, 9 };
float commandeMoteurs[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };

float commandes[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float commandeAxes[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

////////
// Asservissement
////////
float position [6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

float positionVoulue [6] = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};

float angleMax = 10.0f;
float gainProp = 1.0f/angleMax;
float gainInt = 0.0f;
float gainDer = 0.0f;


double t = 0.0f;
double dt = 0.0f;
double t_1 = 0.0f;


float v_max = 10.0;


File errorFile;
const int chipSelect = BUILTIN_SDCARD;


#define voieLacetF 0
#define voieAvanceF 1
#define voieMonteeF 2
#define voieCrabeF 3
#define voieNacelleF 4
#define intModePilotageF 5
#define intModeAsservF 6
#define intPuissanceVentouse 7

#define voieBallastRouliP 0
#define voieTangageP 1
#define voieBallastTangageP 2
#define voieRoulisP 3
#define intModeBallastP 4
#define intMode 5





void blinkLed(unsigned int numFlash = 1) {
  for (unsigned int i = 0; i < numFlash; i++) {
    digitalWrite(ledPinTeensy, HIGH);
    delay(200);
    digitalWrite(ledPinTeensy, LOW);
    delay(200);
  }
}

void Error(int nbFlash, File ErrorFile, const char* message) {
  blinkLed(nbFlash);
  ErrorFile.write(message);
  ErrorFile.flush();
}

void print(File ErrorFile, const char* message) {
  Serial.println(message);
  ErrorFile.write(message);
  ErrorFile.flush();
}

void print(File ErrorFile, int message) {
  Serial.println(message);
  ErrorFile.write(message);
  ErrorFile.flush();
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (x>in_max) { x=in_max; }
  else if (x<in_min) { x=in_min; }
  
  return out_min + (x - in_min) * (out_max - out_min) / (in_max - in_min);
}

void updateDt () {
  t = millis();
  dt = t - t_1;
  t_1 = t;
}

///////////////////////////////////////
// SD
//////////////////////////////////////////
void InitSDCard() {
  Serial.println("Initializing SD card....");
  while (!SD.begin(chipSelect)) {
    Serial.println("Initialization failed!");
    blinkLed(3);
    delay(400);
  }
  blinkLed(1);
  Serial.println("initialization done.");
}

File OpenFile(const char* fichier) {
  File myFile = SD.open(fichier, FILE_WRITE);
  while (!myFile) {
    Serial.println("error opening file");
    blinkLed(4);
    delay(400);
  }
  Serial.println("Ready to receive data from outside source");
  blinkLed(1);
  return myFile;
}

///////////////////////////////////////////////////////////
// SBUS
///////////////////////////////////////////////////////////
bfs::SbusData read_SBUS(bfs::SbusRx* sbus_rx) {
  if ((*sbus_rx).Read()) {
    /* Grab the received data */

    return (*sbus_rx).data();

    

    /* Set the SBUS TX data to the received data */
    //sbus_tx.data(data);
    /* Write the data to the servos */
    //sbus_tx.Write();
  }
}

void print_SBUS_data(bfs::SbusData data) {
  /* Display the received data */
  for (int8_t i = 0; i < data.NUM_CH; i++) {
    Serial.print(data.ch[i]);
    Serial.print("\t");
  }
  /* Display lost frames and failsafe data */
  Serial.print(data.lost_frame);
  Serial.print("\t");
  Serial.println(data.failsafe);
}

///////////////////////////////////
// PWM Moteurs
////////////////////////////////////
void init_PWM_ESC(File ErrorFile) {
  print(ErrorFile, "Initialisation des moteurs...");

  for (int i = 0; i < 8; i++) {
    moteurs[i].attach(pin_moteurs[i]);
  }

  // Étape 1 : Envoyer une impulsion haute pour entrer en mode d'initialisation
  for (int i = 0; i < 8; i++) {
    moteurs[i].writeMicroseconds(2000);
  }
  //Serial.println("Étape 1 : Signal haut envoyé (2000µs)");
  delay(2000);  // Attendre la détection de l'ESC

  // Étape 2 : Envoyer une impulsion basse pour armer l'ESC
  for (int i = 0; i < 8; i++) {
    moteurs[i].writeMicroseconds(1000);
  }
  //Serial.println("Étape 2 : Signal bas envoyé (1000µs)");
  delay(2000);  // Temps pour l'armement

  // Étape 3 : Mettre à 1500µs (neutre)
  //for (int i = 0; i < 8; i++) {
  //moteurs[i].writeMicroseconds(2000);
  //}
  //Serial.println("Étape 3 : Position neutre (1500µs)");
  //delay(50);
  //for (int i = 0; i < 8; i++) {
  //moteurs[i].writeMicroseconds(1000);
  //}

  print(ErrorFile, "Fin de l'initialisation des moteurs.");
}



///////////////////////
// Boucle Moteurs
///////////////////////
void conversion_xyz_moteurs () {
  commandeMoteurs[0] =   commandeAxes[0] - commandeAxes[1] - commandeAxes[5];
  commandeMoteurs[1] =   commandeAxes[0] + commandeAxes[1] + commandeAxes[5];
  commandeMoteurs[2] = - commandeAxes[0] + commandeAxes[1] - commandeAxes[5];
  commandeMoteurs[3] = - commandeAxes[0] - commandeAxes[1] + commandeAxes[5];
  commandeMoteurs[4] =   commandeAxes[2] + commandeAxes[3] + commandeAxes[4];
  commandeMoteurs[5] =   commandeAxes[2] - commandeAxes[3] + commandeAxes[4];
  commandeMoteurs[6] =   commandeAxes[2] + commandeAxes[3] - commandeAxes[4];
  commandeMoteurs[7] =   commandeAxes[2] - commandeAxes[3] - commandeAxes[4];

  for (unsigned int i =0; i<8; i++) {
    moteurs[i].writeMicroseconds(mapFloat(commandeMoteurs[i],-1.0f,1.0f,PWM_MIN,PWM_MAX));
  }
}

void conv_RX_moteurs (bfs::SbusData dataF, bfs::SbusData dataP) {
  commandes[0] = mapFloat((float)dataF.ch[voieAvanceF],  MIN_SBUS, MAX_SBUS, -1.0f, 1.0f);
  commandes[1] = mapFloat((float)dataF.ch[voieCrabeF],   MIN_SBUS, MAX_SBUS, -1.0f, 1.0f);
  commandes[2] = mapFloat((float)dataF.ch[voieMonteeF],  MIN_SBUS, MAX_SBUS, -1.0f, 1.0f);
  commandes[3] = mapFloat((float)dataP.ch[voieTangageP], MIN_SBUS, MAX_SBUS, -1.0f, 1.0f);
  commandes[4] = mapFloat((float)dataP.ch[voieRoulisP],  MIN_SBUS, MAX_SBUS, -1.0f, 1.0f);
  commandes[5] = mapFloat((float)dataF.ch[voieLacetF],   MIN_SBUS, MAX_SBUS, -1.0f, 1.0f);
}

void boucle_moteur(File ErrorFile, bfs::SbusData dataF, bfs::SbusData dataP) {
  
  conv_RX_moteurs (dataF, dataP);

  double erreur = 0.0f;

  switch (dataF.ch[intModeAsservF] / 682) {  // Diviser pour obtenir 0, 1 ou 2
    case 0:
      for (unsigned int i = 0; i<6; i++) {
        commandeAxes[i] = commandes[i];
      }
      break;

    case 1:
      erreur = positionVoulue[3] - position[3];
      commandeAxes[3] = (commandes[3] + erreur * gainProp)/2;
      erreur  = positionVoulue[4] - position[4];
      commandeAxes[4] = (commandes[4] + erreur  * gainProp)/2;

      commandeAxes[0] = commandes[0];
      commandeAxes[1] = commandes[1];
      commandeAxes[2] = commandes[2];

      commandeAxes[5] = commandes[5];
      break;

    default:
      for (unsigned int i = 0; i<6; i++) {
        positionVoulue[i] += v_max * commandes[i] * dt;
        erreur = positionVoulue[i] - position[i];
        commandeAxes[i] = erreur * gainProp;
      }
      
      break;
  }

  conversion_xyz_moteurs();
}


//////////////////////////
// Nacelle
//////////////////////////
void control_nacelle (bfs::SbusData dataF, bfs::SbusData dataP) {
  double vitPWM = mapFloat(dataF.ch[voieNacelleF],MIN_SBUS,MAX_SBUS,PWM_MIN,PWM_MAX);
  servoNacelle1.writeMicroseconds(vitPWM);
  servoNacelle2.writeMicroseconds(vitPWM);


  int val = map(dataF.ch[intPuissanceVentouse], MIN_SBUS, MAX_SBUS, 0, 255);
  analogWrite(ventouse1Mag1,val);
  //analogWrite(ventouse1Mag2,0);
  analogWrite(ventouse2Mag1,val);
  //analogWrite(ventouse2Mag2,0);
}

////////////////////////////
// Pompes
////////////////////////////
void control_ballasts (bfs::SbusData dataF, bfs::SbusData dataP) {

  int vitessePompes [2] = {0,0};

  switch (dataP.ch[intModeBallastP] / 682) {  // Diviser pour obtenir 0, 1 ou 2
    case 0:
      
      break;

    case 1:
      servoEchangeBallast1.writeMicroseconds(2000);
      servoEchangeBallast2.writeMicroseconds(2000);

      vitessePompes [0] = map(dataP.ch[voieBallastRouliP],   MIN_SBUS, MAX_SBUS, -255, 255);
      vitessePompes [1] = map(dataP.ch[voieBallastTangageP], MIN_SBUS, MAX_SBUS, -255, 255);
      break;

    default:
      servoEchangeBallast1.writeMicroseconds(1000);
      servoEchangeBallast2.writeMicroseconds(1000);

      vitessePompes [0] = map(dataP.ch[voieBallastRouliP],   MIN_SBUS, MAX_SBUS, -255, 255);
      vitessePompes [1] = map(dataP.ch[voieBallastTangageP], MIN_SBUS, MAX_SBUS, -255, 255);
      break;
  }

  if (vitessePompes [0]>0) {
    analogWrite(pinPompeBallastRoulis1, vitessePompes[0]);
    analogWrite(pinPompeBallastRoulis2,0);
  }
  else {
    analogWrite(pinPompeBallastRoulis1,0);
    analogWrite(pinPompeBallastRoulis2,-vitessePompes[0]);
  }

  if (vitessePompes [1]>0) {
    analogWrite(pinPompeBallastTangage1, vitessePompes[1]);
    analogWrite(pinPompeBallastTangage2,0);
  }
  else {
    analogWrite(pinPompeBallastTangage1,0);
    analogWrite(pinPompeBallastTangage2,-vitessePompes[1]);
  }
}


void setup() {
  Serial.begin(115200);

  InitSDCard();
  errorFile = OpenFile("errorFile.txt");

  //////////////////
  // OUTPUT
  /////////////////
  pinMode(ledPinTeensy, OUTPUT);
  pinMode(ledPinVerte, OUTPUT);
  pinMode(ledPinRouge, OUTPUT);
  pinMode(ventouse1Mag1, OUTPUT);
  //pinMode(ventouse1Mag2, OUTPUT);
  pinMode(ventouse2Mag1, OUTPUT);
  //pinMode(ventouse2Mag2, OUTPUT);

  /////////////////////////////
  // INPUT
  //////////////////
  pinMode(pinBoutonVert, INPUT);
  pinMode(pinBoutonRouge, INPUT);


  servoNacelle1.attach(pinServoNacelle1);
  //servoNacelle1.writeMicroseconds(1500);
  servoNacelle2.attach(pinServoNacelle2);
  //servoNacelle2.writeMicroseconds(1500);

  servoEchangeBallast1.attach(pinServoEchangeBallast1);
  servoEchangeBallast2.attach(pinServoEchangeBallast2);

  init_PWM_ESC(errorFile);


  //receiver.setMinMax(minMax);

  sbus_rx_Futaba.Begin();
  sbus_rx_ProTronik.Begin();
  //sbus_tx.Begin();


  //attachInterrupt(digitalPinToInterrupt(1), pwmISR, CHANGE);

  delay(1000);

  t_1 = millis();
}



void loop() {
  noInterrupts();  // Désactive les interruptions pour éviter les conflits de lecture
  //uint32_t pwmValue = pulseWidth;
  interrupts();  // Réactive les interruptions

  updateDt();

  dataFutaba = read_SBUS(&sbus_rx_Futaba);
  //print_SBUS_data(dataFutaba);

  dataProTronik = read_SBUS(&sbus_rx_ProTronik);
  //print_SBUS_data(dataProTronik);

  boucle_moteur (errorFile, dataFutaba, dataProTronik);

  control_nacelle (dataFutaba, dataProTronik);
  
  control_ballasts (dataFutaba, dataProTronik);


  //servoEchangeBallast1.write(0);
  //delay(3000);
  //servoEchangeBallast1.write(180);
  //delay(3000);
  //servoEchangeBallast1.write(90);
  //delay(3000);


  //moteurs[0].writeMicroseconds(map(dataFutaba.ch[1], MIN_SBUS, MAX_SBUS, PWM_MIN, PWM_MAX));

  //print(errorFile, dataFutaba.ch[1]);

}
