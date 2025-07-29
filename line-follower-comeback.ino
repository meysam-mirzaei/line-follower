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

#define LED 49
#define PB 48

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

#define MAX (1 << 10) - 1
#define MIN 0
#define MID (MAX + MIN) / 2

int thresholds[15] = {MID, MID, MID, MID, MID, MID, MID, MID, MID, MID, MID, MID, MID, MID, MID};

void setup() {
  debugbegin(9600);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(PB, INPUT_PULLUP);
  if (digitalRead(PB) == LOW) {
    int minimums[15] = {MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX, MAX};
    int maximums[15] = {MIN, MIN, MIN, MIN, MIN, MIN, MIN, MIN, MIN, MIN, MIN, MIN, MIN, MIN, MIN};
    digitalWrite(LED, HIGH);
    while(digitalRead(PB) == LOW);
    digitalWrite(LED, LOW);

    int sensors_w[15] = {analogRead(A0 ), analogRead(A1 ), analogRead(A2 ), analogRead(A3 ),
                         analogRead(A4 ), analogRead(A5 ), analogRead(A6 ), analogRead(A7 ),
                         analogRead(A8 ), analogRead(A9 ), analogRead(A10), analogRead(A11),
                         analogRead(A12), analogRead(A13), analogRead(A14)};
    for (int i = 0; i < 15; i++) {
      if (sensors_w[i] < minimums[i]) {
        minimums[i] = sensors_w[i];
      }
    }

    while(digitalRead(PB) == HIGH);
    digitalWrite(LED, HIGH);

    int sensors_b[15] = {analogRead(A0 ), analogRead(A1 ), analogRead(A2 ), analogRead(A3 ),
                         analogRead(A4 ), analogRead(A5 ), analogRead(A6 ), analogRead(A7 ),
                         analogRead(A8 ), analogRead(A9 ), analogRead(A10), analogRead(A11),
                         analogRead(A12), analogRead(A13), analogRead(A14)};
    for (int i = 0; i < 15; i++) {
      if (sensors_b[i] > maximums[i]) {
        maximums[i] = sensors_b[i];
      }
    }

    for (int i = 0; i < 15; i++) {
      int threshold = (minimums[i] + maximums[i]) / 2;
      EEPROM.update(2 * i, threshold & 0xff);
      EEPROM.update(2 * i + 1, (threshold & 0xff00) >> 8);
    }
    digitalWrite(LED, LOW);
  } else {
    for (int i = 0; i < 15; i++) {
      thresholds[i] = ((EEPROM.read(2 * i + 1)) << 8) + EEPROM.read(2 * i);
    }
  }
}

void loop() {
  float volt = analogRead(A15) * 4.0 * 5.0 / 1024.0;
  debug("Battery Voltage is ");
  debugln(volt);

  bool sensors[15];
  sensors[0]  = analogRead(A0 ) > thresholds[0];
  sensors[1]  = analogRead(A1 ) > thresholds[1];
  sensors[2]  = analogRead(A2 ) > thresholds[2];
  sensors[3]  = analogRead(A3 ) > thresholds[3];
  sensors[4]  = analogRead(A4 ) > thresholds[4];
  sensors[5]  = analogRead(A5 ) > thresholds[5];
  sensors[6]  = analogRead(A6 ) > thresholds[6];
  sensors[7]  = analogRead(A7 ) > thresholds[7];
  sensors[8]  = analogRead(A8 ) > thresholds[8];
  sensors[9]  = analogRead(A9 ) > thresholds[9];
  sensors[10] = analogRead(A10) > thresholds[10];
  sensors[11] = analogRead(A11) > thresholds[11];
  sensors[12] = analogRead(A12) > thresholds[12];
  sensors[13] = analogRead(A13) > thresholds[13];
  sensors[14] = analogRead(A14) > thresholds[14];
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
    MLF(255); 
    MRF(255);
  } else if (sensors[8]) {
    MLF(255); 
    MRF(150);
  } else if (sensors[6]) {
    MLF(150); 
    MRF(255);
  } else if (sensors[9]) {
    MLF(255); 
    MRF(70);
  } else if (sensors[5]) {
    MLF(70); 
    MRF(255);
  } else if (sensors[10]) {
    MLF(255); 
    MRS();
  } else if (sensors[4]) {
    MLS(); 
    MRF(255);
  } else if (sensors[11]) {
    MLF(255); 
    MRB(90);
    } else if (sensors[3]) {
    MLB(90); 
    MRF(255);
  } else if (sensors[12]) {
    MLF(255); 
    MRB(180);
  } else if (sensors[2]) {
    MLB(180); 
    MRF(255);
  } else if (sensors[13]) {
    MLF(255); 
    MRB(225);
  } else if (sensors[1]) {
    MLB(225); 
    MRF(255);
  } else if (sensors[14]) {
    MLF(255); 
    MRB(255);
  } else if (sensors[0]) {
    MLB(255); 
    MRF(255);
  } else {
    
    bool no_line = true;
    for (int i = 0; i < 15; i++) {
      if (sensors[i]) {
        no_line = false;
        break;
      }
    }

    if (no_line) {
      debugln("End of path detected!");

     
      MLS(); MRS();
      delay(300);

      MLF(180); MRB(180);
      delay(1500);

      MLS(); MRS();
      delay(300);

     
      while (true) {
        for (int i = 0; i < 15; i++) {
          sensors[i] = analogRead(A0 + i) > thresholds[i];
        }

       
        if (no_line) {
          debugln("Back to start!");
          MLS(); MRS();
          delay(1000);
          break;
        }

       
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
    }
  }
}