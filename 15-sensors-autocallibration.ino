#include <EEPROM.h>

#define DEBUG
#undef DEBUG

#ifdef DEBUG
#define debugbegin(x) Serial.begin(x)
#define debugln(x)    Serial.println(x)
#define debug(x)      Serial.print(x)
#else
#define debugbegin(x)
#define debugln(x)
#define debug(x)
#endif

#define EN1 2
#define EN2 3
#define IN3 50
#define IN4 51
#define IN1 52
#define IN2 53
#define CALIB_BUTTON 22

#define MRF(pwm) \
  digitalWrite(IN1, LOW); \
  digitalWrite(IN2, HIGH); \
  analogWrite(EN1, pwm);
#define MRB(pwm) \
  digitalWrite(IN1, HIGH); \
  digitalWrite(IN2, LOW); \
  analogWrite(EN1, pwm);
#define MRS() \
  digitalWrite(IN1, HIGH); \
  digitalWrite(IN2, HIGH); \
  analogWrite(EN1, 255);
#define MLF(pwm) \
  digitalWrite(IN3, HIGH); \
  digitalWrite(IN4, LOW); \
  analogWrite(EN2, pwm);
#define MLB(pwm) \
  digitalWrite(IN3, LOW); \
  digitalWrite(IN4, HIGH); \
  analogWrite(EN2, pwm);
#define MLS() \
  digitalWrite(IN3, HIGH); \
  digitalWrite(IN4, HIGH); \
  analogWrite(EN2, 255);

const int sensorPins[15] = {  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14  };

int sensorMin[15];
int sensorMax[15];
int Threshold[15];
int Values[15];

void setup() {
  debugbegin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(CALIB_BUTTON, INPUT);

   readThresholdsFromEEPROM();
   }



void loop() {
  if (digitalRead(CALIB_BUTTON) == HIGH) {
    calibrateSensors();
    saveThresholdsToEEPROM();
    delay(1000);
  }

  readSensors();

  bool sensors[15];
  sensors[0]  = Values[0] > Threshold[0];
  sensors[1]  = Values[1] > Threshold[1];
  sensors[2]  = Values[2] > Threshold[2];
  sensors[3]  = Values[3] > Threshold[3];
  sensors[4]  = Values[4] > Threshold[4];
  sensors[5]  = Values[5] > Threshold[5];
  sensors[6]  = Values[6] > Threshold[6];
  sensors[7]  = Values[7] > Threshold[7];
  sensors[8]  = Values[8] > Threshold[8];
  sensors[9]  = Values[9] > Threshold[9];
  sensors[10] = Values[10] > Threshold[10];
  sensors[11] = Values[11] > Threshold[11];
  sensors[12] = Values[12] > Threshold[12];
  sensors[13] = Values[13] > Threshold[13];
  sensors[14] = Values[14] > Threshold[14];
  debug((sensors[0])? "1" : "0");
  debug((sensors[1])? "1" : "0");
  debug((sensors[2])? "1" : "0");
  debug((sensors[3])? "1" : "0");
  debug((sensors[4])? "1" : "0");
  debug((sensors[5])? "1" : "0");
  debug((sensors[6])? "1" : "0");
  debug((sensors[7])? "1" : "0");
  debug((sensors[8])? "1" : "0");
  debug((sensors[9])? "1" : "0");
  debug((sensors[10])? "1" : "0");
  debug((sensors[11])? "1" : "0");
  debug((sensors[12])? "1" : "0");
  debug((sensors[13])? "1" : "0");
  debug((sensors[14])? "1" : "0");
  debugln("");


  if (sensors[7]) {
    MLF(255)
    MRF(255)
  } else if (sensors[8]) {
    MLF(255)
    MRF(150)
  } else if (sensors[6]) {
    MLF(150)
    MRF(255)
  } else if (sensors[9]) {
    MLF(255)
    MRF(70)
  } else if (sensors[5]) {
    MLF(70)
    MRF(255)
  } else if (sensors[10]) {
    MLF(255)
    MRS()
  } else if (sensors[4]) {
    MLS()
    MRF(255)
  } else if (sensors[11]) {
    MLF(255)
    MRB(90)
  } else if (sensors[3]) {
    MLB(90)
    MRF(255)
  } else if (sensors[12]) {
    MLF(255)
    MRB(180)
  } else if (sensors[2]) {
    MLB(180)
  MRF(255)
  } else if (sensors[13]) {
    MLF(255)
    MRB(225)
  } else if (sensors[1]) {
    MLB(225)
    MRF(255)
  } else if (sensors[14]) {
    MLF(255)
    MRB(255)
  } else if (sensors[0]) {
    MLB(255)
    MRF(255)
  }
}

void readSensors() {
  for (int i = 0; i < 15; i++) {
    Values[i] = analogRead(sensorPins[i]);
  }
}

void calibrateSensors() {
  for (int i = 0; i < 15; i++) {
    sensorMin[i] = 1023;
    sensorMax[i] = 0;
  }

  while (digitalRead(CALIB_BUTTON) == HIGH) {
    for (int i = 0; i < 15; i++) {
      int val = analogRead(sensorPins[i]);
      if (val < sensorMin[i]) sensorMin[i] = val;
      if (val > sensorMax[i]) sensorMax[i] = val;
    }
    delay(5);
  }

  for (int i = 0; i < 15; i++) {
    if (sensorMin[i] != sensorMax[i]) {
      Threshold[i] = (sensorMin[i] + sensorMax[i]) / 2;
    } else {
      Threshold[i] = sensorMin[i]; 
      }
   }
}

void saveThresholdsToEEPROM() {
  for (int i = 0; i < 15; i++) {
    EEPROM.put(i * sizeof(int), Threshold[i]);
  }
}

void readThresholdsFromEEPROM() {
  for (int i = 0; i < 15; i++) {
    EEPROM.get(i * sizeof(int), Threshold[i]);

    if (Threshold[i] < 10 || Threshold[i] > 1010) {
      Threshold[i] = 600;
    }
  }
}

