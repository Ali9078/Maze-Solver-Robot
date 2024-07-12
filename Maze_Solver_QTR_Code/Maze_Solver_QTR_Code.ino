#include <QTRSensors.h>

#define Kp 0.05
#define Kd 0.52
#define Ki 0.002
#define rightMotor1 A1
#define rightMotor2 A0
#define rightMotorPWM 9
#define leftMotor1 A3
#define leftMotor2 A4
#define leftMotorPWM 10
#define motorPower A2

boolean replacement = false;

boolean found_dry_intersect = false;

int MaxSpeed = 245;
int BaseSpeed = 225;

int inc = 0;

String path = "";
// booleans for adding to path
boolean straight = false;
boolean goback = false;
boolean left = false;


boolean dry = false;
boolean pid =  false;
boolean turn = false;
boolean maze = false;


boolean confirm_intersect = false;
boolean founduturn = false;
boolean mazeright;
boolean mazeleft;
boolean left = false;
boolean leftintersect;
boolean rightintersect;



QTRSensors qtr;
QTRSensors qtrnew;

const uint8_t SensorCount = 7;
uint16_t sensorValues[SensorCount];
uint16_t newval[SensorCount];
uint16_t mazeval[SensorCount];



float lastError = 0;

void setup()
{
  // configure the sensors
  pinMode(13,HIGH);
  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]) {
    8, 7, 6, 5, 4, 3, 2
  }, SensorCount);
  qtr.setEmitterPin(2);

  qtrnew.setTypeRC();
  qtrnew.setSensorPins((const uint8_t[]) {
    8, 7, 6, 5, 4, 3, 2
  }, SensorCount);
  qtrnew.setEmitterPin(2);


  pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(motorPower, OUTPUT);
  pinMode(12, INPUT);
  pinMode(11, INPUT);
  delay(500);

  digitalWrite(motorPower, HIGH);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 200; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  Serial.begin(9600);
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  stopit();
  Serial.println();
  Serial.println();
  delay(1000);
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhiteWhite() instead)
  uint16_t position = qtr.readLineWhite(sensorValues);


  while (true) {

    int btn1 = digitalRead(12);
    if (btn1 == 1) {
      delay(2000);
      dry = true;
      pid = true;
    }

    while (leftintersect == true && pid == false) {
      leftintersection();

    }
    while (mazeleft == true && pid == false) {
      leftmaze();

    }
    while (founduturn == true) {
      takeuturn();
    }
    while (confirm_intersect == true) {
      mazeintersect();
    }
    while (found_dry_intersect == true) {
      dryintersect();
    }
    while (rightintersect == true) {
      rightintersection();

    }
    while (mazeright == true) {
      rightmaze();

    }

    while (pid == false && leftintersect == false && replacement == true) {
      for (int x = 0; x < 4; x++)
      {
        path.replace("LUR", "U");
        path.replace("LUS", "R");
        path.replace("LUL", "S");
        path.replace("SUL", "R");
        path.replace("SUS", "U");
        path.replace("RUL", "U");
        if (x == 3) {
          Serial.println(path);
          delay(2000);
          digitalWrite(13,LOW);
          replacement = false;
          break;
        }
      }
    }

    while (pid == true) {

      path.replace("LUR", "U");
      path.replace("LUS", "R");
      path.replace("LUL", "S");
      path.replace("SUL", "R");
      path.replace("SUS", "U");
      path.replace("RUL", "U");
      position = qtr.readLineWhite(sensorValues);
      /*for (int m = 0; m < SensorCount; m++) {
        Serial.print(sensorValues[m]);
        Serial.print('\t');
        }*/
      Serial.println(path);
      /*Serial.print(path[inc]);
        Serial.print('\t');
        Serial.println(inc);*/
      int error = position - 3000;
      int error1 = error - lastError;
      int error2 = (2.0 / 3.0) * error2 + error ;
      int motorSpeed = Kp * error + Kd * error1 + Ki * error2;
      int rightMotorSpeed = BaseSpeed - motorSpeed;
      int leftMotorSpeed = BaseSpeed + motorSpeed;
      if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
      if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
      if (rightMotorSpeed < 0)rightMotorSpeed = 0;
      if (leftMotorSpeed < 0)leftMotorSpeed = 0;
      lastError = error;
      analogWrite(rightMotorPWM, rightMotorSpeed);
      analogWrite(leftMotorPWM, leftMotorSpeed);


      /* if (sensorValues[1] < 200 && sensorValues[2] < 200 && sensorValues[3] < 200 && sensorValues[4] < 200 && sensorValues[5] < 200) {
         if (dry == true) {
           left = true;
           stopit();
           found_dry_intersect= true;
           pid = false;
         }
         else if (maze == true) {
           got = true;
           stopit();
           confirm_intersect = true;
           pid = false;
         }
        }*/

      if ((sensorValues[0] < 750 || sensorValues[6] < 750) && turn == true) {
        if (sensorValues[0] < 750 && sensorValues[6] < 750) {
          if (dry == true) {
            left = true;
            stopit();
            goback = false;
            found_dry_intersect = true;
            pid = false;
          }
          else if (maze == true) {
            stopit();
            goback = false;
            confirm_intersect = true;
            pid = false;
          }

        }

        else if ((sensorValues[6] < 250) && sensorValues[0] > 950) {
          if (dry == true) {

            stopit();
            goback = false;
            rightintersect = true;
            pid = false;
          }
          else if (maze == true) {
            stopit();
            goback = false;
            mazeright = true;
            pid = false;
          }

        }
        else if (sensorValues[0] < 250 && sensorValues[6] > 950) {
          if (dry == true) {
            stopit();
            goback = false;
            leftintersect = true;
            pid = false;
          }
          else if (maze == true) {
            stopit();
            goback = false;
            mazeleft = true;
            pid = false;
          }

        }
      }




      else if (sensorValues[0] > 950 && sensorValues[1] > 950 && sensorValues[2] > 950 && sensorValues[3] > 950 && sensorValues[4] > 950 && sensorValues[5] > 950 && sensorValues[6] > 950) {
        turn = false;
        founduturn = true;
        pid = false;
      }

      else {
        turn = true;
        if (straight == true) {
          path = path + "S";
          straight = false;
        }
        else if (left == true) {
          path = path + "L";
          left = false;
        }
        else if (goback == true) {
          path = path + "U";
          goback = false;
        }
        digitalWrite(rightMotor2, LOW);
        digitalWrite(rightMotor1, HIGH);
        digitalWrite( leftMotor1, LOW);
        digitalWrite( leftMotor2, HIGH);
        analogWrite(rightMotorPWM, rightMotorSpeed);
        analogWrite(leftMotorPWM, leftMotorSpeed);

      }

    }

    int btn2 = digitalRead(11);
    if (btn2 == 1) {
      delay(2000);
      inc = 0;
      maze = true;
      pid = true;
    }

  }
}




void forward() {
  analogWrite(rightMotorPWM, 140);
  analogWrite(leftMotorPWM, 140);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite( leftMotor1, LOW);
  digitalWrite( leftMotor2, HIGH);

}
void backward() {
  analogWrite(rightMotorPWM, 140);
  analogWrite(leftMotorPWM, 140);
  digitalWrite(rightMotor2, HIGH);
  digitalWrite(rightMotor1, LOW);
  digitalWrite( leftMotor1, HIGH);
  digitalWrite( leftMotor2, LOW);

}
void rightturn() {
  analogWrite(rightMotorPWM, 155);
  analogWrite(leftMotorPWM, 155);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite( leftMotor1, HIGH);
  digitalWrite( leftMotor2, LOW);

}
void uturn() {
  analogWrite(rightMotorPWM, 180);
  analogWrite(leftMotorPWM, 160);
  digitalWrite(rightMotor2, LOW);
  digitalWrite(rightMotor1, HIGH);
  digitalWrite( leftMotor1, HIGH);
  digitalWrite( leftMotor2, LOW);

}

void normalleft() {
  analogWrite(rightMotorPWM, 155);
  analogWrite(leftMotorPWM, 155);
  digitalWrite(rightMotor2, HIGH);
  digitalWrite(rightMotor1, LOW);
  digitalWrite( leftMotor1, LOW);
  digitalWrite( leftMotor2, HIGH);

}

void stopit() {
  digitalWrite(rightMotor2, LOW);
  digitalWrite(rightMotor1, LOW);
  digitalWrite( leftMotor1, LOW);
  digitalWrite( leftMotor2, LOW);

}

void dryintersect() {
  delay(10);
  stopit();
  qtrnew.read(newval);

  if (newval[3] < 1400) {
    left = true;

  }
  while (true) {
    qtrnew.read(newval);
    if (newval[0] < 1400 && newval[1] < 1400 && newval[2] < 1400 && newval[3] < 1400 && newval[4]  < 1400 && newval[5] < 1400 && newval[6] < 1400) {
      forward();
      delay(30);
      normalleft();
      delay(120);
      stopit();
      Serial.println(path);
      path = path + "E";
      digitalWrite(13,HIGH);
      replacement = true;
      pid = false;
      dry = false;
      found_dry_intersect = false;
      break;
    }
    else {
      normalleft();
      if (newval[1] < 1400) {
        stopit();
        pid = true;
        found_dry_intersect = false;
        break;
      }
    }
  }
}
void takeuturn() {
  while (true) {
    uturn();
    goback = true;

    qtr.read(newval);
    if (newval[5] < 1400) {
      stopit();
      pid = true;
      founduturn = false;
      break;
    }
  }
}
void leftintersection() {
  delay(10);
  stopit();
  qtrnew.read(newval);

  if (newval[3] < 1400 || newval[6] < 1400) {
    left = true;

  }
  while (true) {
    qtrnew.read(newval);
    if (newval[0] < 1100 && newval[1] < 1100 && newval[2] < 1100 && newval[3] < 1100 && newval[4]  < 1100 && newval[5] < 1100 && newval[6] < 1100) {
      forward();
      delay(30);
      normalleft();
      delay(120);
      stopit();
      Serial.println(path);
      path = path + "E";
      digitalWrite(13,HIGH);
      replacement = true;
      pid = false;
      dry = false;
      leftintersect = false;
      break;
    }
    else {
      normalleft();
      if (newval[1] < 1400 ) {
        stopit();
        pid = true;
        leftintersect = false;
        break;
      }
    }
  }
}
void rightintersection() {
  delay(10);
  stopit();
  qtrnew.read(newval);
  if (newval[3] < 1400) {
    straight = true;
    forward();
    delay(10);
    pid = true;
    rightintersect = false;
  }
  else {

    while (true) {
      qtrnew.read(newval);
      rightturn();
      if (newval[5] < 1400 ) {
        stopit();
        pid = true;
        rightintersect = false;
        break;
      }

    }
  }
}
void rightmaze() {
  delay(10);
  stopit();
  qtrnew.read(newval);
  if (newval[3] < 1400) {
    switch (path[inc]) {
      case 'R':
        while (true) {
          qtrnew.read(newval);
          rightturn();
          if (newval[5] < 1400) {
            stopit();
            inc += 1;
            pid = true;
            mazeright = false;
            break;
          }
        }
        break;
      case 'L':
        while (true) {
          qtrnew.read(newval);
          normalleft();
          if (newval[1] < 1400) {
            stopit();
            inc += 1;
            pid = true;
            mazeright = false;
            break;
          }
        }
        break;
      case 'S':
        forward();
        delay(40);
        inc += 1;
        pid = true;
        mazeright = false;
        break;
    }

  }
  else {

    while (true) {
      qtrnew.read(newval);
      rightturn();
      if (newval[5] < 1400) {
        stopit();
        pid = true;
        mazeright = false;
        break;
      }

    }
  }
}
void leftmaze() {
  delay(10);
  stopit();
  qtrnew.read(newval);
  if (newval[3] < 1400 || newval[6] < 1400) {
    switch (path[inc]) {
      case 'R':
        while (true) {
          qtrnew.read(newval);
          rightturn();
          if (newval[5] < 1400) {
            stopit();
            inc += 1;
            pid = true;
            mazeleft = false;
            break;
          }
        }
        break;
      case 'L':
        while (true) {
          qtrnew.read(newval);
          normalleft();
          if (newval[1] < 1400) {
            stopit();
            inc += 1;
            pid = true;
            mazeleft = false;
            break;
          }
        }
        break;
      case 'S':
        forward();
        delay(40);
        inc += 1;
        pid = true;
        mazeleft = false;
        break;
      case 'E':
      digitalWrite(13,HIGH);
        stopit();
        pid = false;
        mazeleft = false;
        break;
    }


  }
  else {

    while (true) {
      qtrnew.read(newval);
      normalleft();
      if (newval[1] < 1400) {
        stopit();
        pid = true;
        mazeleft = false;
        break;
      }

    }
  }
}
void mazeintersect() {
  delay(10);
  stopit();
  if (true) {
    switch (path[inc]) {
      case 'R':
        while (true) {
          qtrnew.read(newval);
          rightturn();
          if (newval[5] < 1400) {
            stopit();
            inc += 1;
            pid = true;
            confirm_intersect = false;
            break;
          }
        }
        break;
      case 'L':
        while (true) {
          qtrnew.read(newval);
          normalleft();
          if (newval[1] < 1400) {
            stopit();
            inc += 1;
            pid = true;
            confirm_intersect = false;
            break;
          }
        }
        break;
      case 'S':
        forward();
        delay(40);
        inc += 1;
        pid = true;
        confirm_intersect = false;
        break;
      case 'E':
        digitalWrite(13,HIGH);
        stopit();
        maze = false;
        pid = false;
        confirm_intersect = false;
        break;
    }

  }

}
