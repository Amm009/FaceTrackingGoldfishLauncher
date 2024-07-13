/*
Ammon Connell
7/18/2023
Face tracking Goldfish launcher:
This project uses a face sensor from Useful Sensors to relay information about where a face is to an Arduino.
Using this information, my code will aim horizontally and vertically in order to hit where it thinks is the center
of the face.
*/

#include <Servo.h>
#include <TouchScreen.h>
#include <MCUFRIEND_kbv.h>
#include <Wire.h>
#include <person_sensor.h>

//Defining pins for the touchscreen library... Lines 7-33 is all just taken from example code.
const int XP = 8, XM = A2, YP = A3, YM = 9;  //ID=0x9341
const int TS_LEFT = 936, TS_RT = 86, TS_TOP = 68, TS_BOT = 904;

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 364);

MCUFRIEND_kbv tft;

//Defining pins for the MCUFRIEND_kbv library
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
#define LCD_RESET A4

//Color Definitons
#define BLACK 0x0000
#define BLUE 0x001F
#define GREY 0xCE79
#define LIGHTGREY 0xDEDB
#define RED 0xF800
#define GREEN 0x07E0
#define CYAN 0x07FF
#define MAGENTA 0xF81F
#define YELLOW 0xFFE0
#define WHITE 0xFFFF

//Booleans that are in the code
bool beginpressed = false;
bool confirmpressed = false;
bool nofacerun = false;
bool distgood = false;
bool beambroken = false;
bool aimed = false;
bool abrun = false;
bool rootfailure = false;
bool firstroot = false;
bool rootfound = false;
bool secondroot = false;
bool bpositive = false;
bool anglegood = true;
bool errorgood = false;
bool maxangle = false;

//Integers for all values in the code
int numgoldfish = 1;
int nofaceint = 10;
int facehorizontal;
int Hservocommand = 90;
int iter;
int tc;
int bh;
float distance;
float bs;
float Kp = 0.1;
float Ki = 0.005;
float Kd = 0.05;
float e = 0;
float eold = 0;
float de = 0;
float t = 0.1;
float height;
float a = 0;
float b = 0;
float c;
float tilteqr;
long E = 0;

//Naming the servos
Servo horizontal;
Servo tilt;
Servo fire;

void setup() {
  //Setting up servos, the screen, the serial communication, the i2c communication,
  //and the hardware interrupt
  Serial.begin(9600);
  horizontal.attach(48);
  tilt.attach(49);
  fire.attach(50);
  horizontal.write(Hservocommand);
  tilt.write(180);
  Wire.begin();
  uint16_t identifier = tft.readID();
  tft.reset();
  tft.begin(identifier);
  tft.setRotation(1);
  startscreen();
  pinMode(18, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(18), beambreak, FALLING);
}

void loop() {
  //Setting all of the servos to their standby position.
  fire.write(0);
  tilt.write(180);
  horizontal.write(Hservocommand);

  //This command reads where you are touching the screen
  TSPoint p = ts.getPoint();

  //This if statement says that if the begin button has not been pressed and the user has pressed the begin button,
  //show the GF select menu and flag that the user has pressed the begin button.
  if (beginpressed == false && p.z > ts.pressureThreshhold && p.x > 494 && p.x < 654 && p.y > 324 && p.y < 658) {
    GFselectmain();
    GFselectnum();
    beginpressed = true;
  }

  //This if statement says that if the begin button has been pressed and the confirm button has not been pressed, and the user has pressed
  //the cancel button, reset to the beginning.
  if (beginpressed == true && confirmpressed == false && p.z > ts.pressureThreshhold && p.x > 770 && p.x < 876 && p.y > 657 && p.y < 888) {
    reset();
  }

  //If the plus is pressed add one to the numgoldfish int and print the change
  if (beginpressed == true && confirmpressed == false && p.z > ts.pressureThreshhold && p.x > 390 && p.x < 488 && p.y > 460 && p.y < 535) {
    if (numgoldfish <= 2) {
      numgoldfish = numgoldfish + 1;
    } else {
    }
    GFselectnum();
    delay(200);
  }

  //If the minus is pressed subtract one to the numgoldfish int and print the change
  if (beginpressed == true && confirmpressed == false && p.z > ts.pressureThreshhold && p.x > 678 && p.x < 780 && p.y > 460 && p.y < 535) {
    if (numgoldfish >= 2) {
      numgoldfish = numgoldfish - 1;
    } else {
    }
    GFselectnum();
    delay(200);
  }

  //This if statement says that if the begin button has been pressed, and the user has pressed
  //the confirm button, show the searching screen and flag the confirm button as having been pressed.
  if (beginpressed == true && confirmpressed == false && p.z > ts.pressureThreshhold && p.x > 770 && p.x < 876 && p.y > 86 && p.y < 320) {
    searching();
    delay(500);
    confirmpressed = true;
  }

  //If Confirm is pressed and the distance to the users face is too close, enter this loop...
  if (beginpressed == true && confirmpressed == true && distgood == false) {

    person_sensor_results_t results = {};

    //Person sensor not found
    if (!person_sensor_read(&results)) {
      Serial.println("No person sensor results found on the i2c bus");
      delay(200);
      return;
    }

    if (results.num_faces == 0) {
      if (nofacerun == false) {
        noface();
        nofacerun = true;
      }

      nofacenum();
      delay(1000);

      //Timer until reset if no face is found
      if (nofaceint > 0) {
        nofaceint = nofaceint - 1;
      }

      if (nofaceint == 0) {
        reset();
      }

      //If a face is detected, estimate the distance to the users face
    } else {
      const person_sensor_face_t* face = &results.faces[0];
      bs = face->box_bottom - face->box_top;
      distance = 5.7237 * pow(2.71828, -0.024 * bs);
      nofacerun = false;
      nofaceint = 10;
      facefound();
      delay(1000);

      if (distance > 1.2) {
        gooddist();
        delay(500);
        approxdist();
        delay(1000);
        distgood = true;

      } else {
        tooclose();
        delay(1000);
      }
    }
  }

  //If confirm is pressed and the distance to the users face is good, enter this loop...
  if (beginpressed == true && confirmpressed == true && distgood == true) {

    person_sensor_results_t results = {};

    if (!person_sensor_read(&results)) {
      Serial.println("No person sensor results found on the i2c bus");
      delay(200);
      return;
    }

    if (results.num_faces == 0) {
      if (nofacerun == false) {
        noface();
        nofacerun = true;
        aimed = false;
      }

      nofacenum();
      delay(1000);

      if (nofaceint > 0) {
        nofaceint = nofaceint - 1;
      }

      if (nofaceint == 0) {
        reset();
      }

      //If a face is detected, aim horizontally
    } else {
      const person_sensor_face_t* face = &results.faces[0];
      facehorizontal = face->box_right - ((face->box_right - face->box_left) / 2);

      if (aimed == false) {
        aiming();
        aimed = true;
        nofacerun = false;
        nofaceint = 10;
      }

      //To be honest, I pretty much just copied this section from the PID lab...
      e = 127 - facehorizontal;
      E = E + e;
      de = e - eold;
      eold = e;

      //Calculating the command to send to the horizontal servo
      Hservocommand = Hservocommand + Kp * e + Ki * E + Kd * de;

      //Making sure the servo doesn't turn too far
      if (Hservocommand < 50) {
        Hservocommand = 50;
      }

      if (Hservocommand > 130) {
        Hservocommand = 130;
      }

      horizontal.write(Hservocommand);
      delay(200);
    }

    //Making sure that the error is actually within tolerance and not just a fluke...
    if (e < 3 && e > -3) {
      delay(1000);
      person_sensor_results_t results = {};

      if (!person_sensor_read(&results)) {
        Serial.println("No person sensor results found on the i2c bus");
        delay(200);
        return;
      } else {
        const person_sensor_face_t* face = &results.faces[0];
        facehorizontal = face->box_right - ((face->box_right - face->box_left) / 2);
        e = 127 - facehorizontal;
      }

      if (e < 3 && e > -3) {
        errorgood = true;
      }
    }

    //If there is a goldfish to shoot and the horizontal error is small, enter this loop...
    if (numgoldfish > 0 && e < 3 && e > -3 && errorgood == true) {

      person_sensor_results_t results = {};

      if (!person_sensor_read(&results)) {
        Serial.println("No person sensor results found on the i2c bus");
        delay(200);
        return;
      }

      if (results.num_faces == 0) {
        if (nofacerun == false) {
          noface();
          nofacerun = true;
        }

        nofacenum();
        delay(1000);

        if (nofaceint > 0) {
          nofaceint = nofaceint - 1;
        }

        if (nofaceint == 0) {
          reset();
        }
      } else {
        nofacerun = false;
        holdstill();
        delay(3000);
        distance = 0;

        const person_sensor_face_t* face = &results.faces[0];
        bs = face->box_bottom - face->box_top;
        bh = face->box_top + (face->box_bottom - face->box_top) / 2;
        distance = 5.7237 * pow(2.71828, -0.024 * bs);

        //An array of eq to calculate height based on distance and box height (bh)...
        if (distance >= 1 && distance < 1.3) {
          height = (bh - 121.14) / -161.14;
        }
        if (distance >= 1.3 && distance < 1.8) {
          height = (bh - 125.48) / -117.14;
        }
        if (distance >= 1.8 && distance < 2.3) {
          height = (bh - 123.86) / -70.857;
        }
        if (distance >= 2.3 && distance < 2.8) {
          height = (bh - 125.71) / -49.714;
        }
        if (distance >= 2.8) {
          height = (bh - 124.5) / -41.029;
        }

        Serial.println("Height: " + String(height));
        Serial.println("Distance: " + String(distance));

        //This while loop does the root bisection method on the projectile motion eq to calculate
        //what angle to shoot the goldfish at...
        while (rootfailure == false && rootfound == false) {
          //Getting a value for a that results in a negative eq and vice versa for b...
          if (abrun == false) {
            while (tilteq(a, distance, height) < 0) {
              a = a + 1;
              Serial.println("In a low loop");
            }
            while (tilteq(b, distance, height) > 0) {
              b = b + 1;
              Serial.println("In b low loop");
            }
            abrun = true;
            firstroot = true;
          }

          //These two if statements make it so that it chooses the higher, longer
          //shot rather than just shooting straighta t your face if possible
          //(it will basically never do this because in the way that I designed it, the angle is limited to 70 degrees).
          if (firstroot == true && anglegood == true) {
            while (tilteq(b, distance, height) < 0) {
              b = b + 1;
            }
            bpositive = true;
            firstroot = false;
          }

          if (bpositive == true && anglegood == true) {
            while (tilteq(b, distance, height) > 0) {
              b = b + 1;
              Serial.println("In b high loop");
            }
            secondroot = true;
            bpositive = false;
          }

          //Picking the value in the middle and then assigning to a or b depending on the sign...
          c = (a + b) / 2;

          if (tilteq(c, distance, height) > 0) {
            a = c;
          }
          if (tilteq(c, distance, height) < 0) {
            b = c;
          }

          Serial.println("iter: " + String(iter) + " f(a) * f(b): " + String(tilteq(a, distance, height) * tilteq(b, distance, height)) + " c: " + String(c) + " tilteq: " + String(tilteq(b, distance, height)));

          //If the angle is greater than the 75 degree limit, go back and solve it so that it will shoot directly at your face.
          //(It is set up so that the first option is to shoot the higher, longer shot if possible)
          if (c * (180 / 3.14159) > 75) {
            if (maxangle == true) {
              reset();
            }
            anglegood = false;
            rootfound = false;
            abrun = false;
            maxangle = true;
            a = 0;
            b = 0;
          }

          //If the projectile motion eq is close to zero, declare that the root has been found!
          if (tilteq(c, distance, height) < 0.01 && tilteq(c, distance, height) > -0.01) {
            Serial.println("tilteq: " + String(tilteq(c, distance, height)));
            rootfound = true;
          }

          iter = iter + 1;

          //If after 100 iterations, a root still cannot be found, display a failure screen...
          if (iter == 100) {
            rootfailure = true;
            r00tfailure();
            delay(2000);
          }
        }
      }

      //If a root is successfully found, point the servo to that angle and fire!
      if (rootfailure == false) {
        float cd = c * (180 / 3.14159);
        tc = map(cd, 0, 75, 180, 63);
        Serial.println("cd:  " + String(cd) + "  tc: " + String(tc));
        firing();
        GFselectnumred();
        delay(3000);
        tilt.write(tc);
        delay(1000);
        fire.write(0);
        delay(1500);
        fire.write(180);
        delay(1500);

        //If the beam is broken, reduce the number of goldfish remaining by one...
        //If not, display a failure screen and restart
        if (beambroken == true) {
          numgoldfish = numgoldfish - 1;
          beambroken = false;
          aimed = false;
        } else {
          launchfailure();
          delay(2000);
          reset();
        }

        //If the number of goldfish has reached 0, display a success message and restart...
        if (numgoldfish == 0) {
          success();
          delay(2000);
          reset();
        }
      } else {
        reset();
      }
    }
  }
}

//All of the functions for this program are below...

void GFselectmain() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(WHITE);
  tft.setTextSize(2);
  tft.setCursor(40, 10);
  tft.setTextColor(RED);
  tft.print("Select the number of");
  tft.setCursor(20, 30);
  tft.print("Goldfish you would like");
  tft.setCursor(120, 50);
  tft.print("(Max 3)");
  tft.setTextSize(5);
  tft.setCursor(150, 80);
  tft.print("+");
  tft.setCursor(150, 160);
  tft.print("-");
  tft.fillRoundRect(5, 190, 90, 30, 5, GREEN);
  tft.fillRoundRect(225, 190, 90, 30, 5, RED);
  tft.setCursor(8, 197);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.print("Confirm");
  tft.setCursor(236, 197);
  tft.print("Cancel");
}

void GFselectnum() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillRect(140, 110, 60, 60, WHITE);
  tft.setTextSize(4);
  tft.setCursor(153, 125);
  tft.setTextColor(RED);
  tft.print(String(numgoldfish));
}

void GFselectnumred() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillRect(150, 150, 60, 60, RED);
  tft.setTextSize(4);
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.setCursor(8, 130);
  tft.print("GF Remaining:");
  tft.setCursor(153, 170);
  tft.print(String(numgoldfish));
}

void sspressed() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillRoundRect(100, 110, 125, 45, 5, BLACK);
  tft.setCursor(105, 115);
  tft.setTextColor(WHITE);
  tft.print("Begin");
  delay(250);
  tft.fillRoundRect(100, 110, 125, 45, 5, RED);
  tft.setCursor(105, 115);
  tft.setTextColor(WHITE);
  tft.print("Begin");
  delay(250);
}

void startscreen() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(WHITE);
  tft.setCursor(10, 10);
  tft.setTextColor(RED);
  tft.setTextSize(3);
  tft.print("Goldfish Launcher");
  tft.setTextSize(4);
  tft.fillRoundRect(100, 110, 125, 45, 5, RED);
  tft.setCursor(105, 115);
  tft.setTextColor(WHITE);
  tft.print("Begin");
}

void searching() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(WHITE);
  tft.setTextColor(RED);
  tft.setTextSize(4);
  tft.setCursor(50, 95);
  tft.print("Searching");
  tft.setCursor(22, 130);
  tft.print("for face...");
}

void facefound() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(WHITE);
  tft.setTextColor(RED);
  tft.setTextSize(4);
  tft.setCursor(35, 100);
  tft.print("Face found!");
}

void tooclose() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(WHITE);
  tft.setTextColor(RED);
  tft.setTextSize(4);
  tft.setCursor(40, 100);
  tft.print("Too close!");
}

void holdstill() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(WHITE);
  tft.setTextColor(RED);
  tft.setTextSize(3);
  tft.setCursor(35, 40);
  tft.print("Hold still and");
  tft.setCursor(40, 70);
  tft.print("look directly");
  tft.setCursor(20, 100);
  tft.print("into the camera!");
  tft.setCursor(60, 150);
  tft.print("Calculating");
  tft.setCursor(25, 180);
  tft.print("launch angle...");
}

void noface() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(WHITE);
  tft.setTextColor(RED);
  tft.setTextSize(4);
  tft.setCursor(80, 50);
  tft.print("No face");
  tft.setCursor(60, 90);
  tft.print("detected!");
}

void nofacenum() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillRect(140, 140, 100, 50, WHITE);
  tft.setTextColor(RED);
  tft.setTextSize(4);
  tft.setCursor(150, 150);
  tft.print(String(nofaceint));
}

void gooddist() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(WHITE);
  tft.setTextColor(GREEN);
  tft.setTextSize(3);
  tft.setCursor(40, 70);
  tft.print("Distance good!");
}

void aiming() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(WHITE);
  tft.setTextColor(RED);
  tft.setTextSize(4);
  tft.setCursor(55, 90);
  tft.print("Aiming...");
}

void firing() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(RED);
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.setCursor(90, 80);
  tft.print("FIRING");
}

void launchfailure() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(RED);
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.setCursor(90, 80);
  tft.print("LAUNCH");
  tft.setCursor(80, 130);
  tft.print("FAILURE");
}

void r00tfailure() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(RED);
  tft.setTextColor(WHITE);
  tft.setTextSize(4);
  tft.setCursor(120, 80);
  tft.print("ROOT");
  tft.setCursor(80, 130);
  tft.print("FAILURE");
}

void success() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillScreen(WHITE);
  tft.setTextColor(GREEN);
  tft.setTextSize(4);
  tft.setCursor(80, 100);
  tft.print("SUCCESS!");
}

void approxdist() {
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  tft.fillRect(35, 95, 100, 40, WHITE);
  tft.setTextColor(GREEN);
  tft.setTextSize(3);
  tft.setCursor(55, 100);
  tft.print("Dist approx: ");
  tft.setCursor(100, 130);
  tft.print(String(distance) + "ft");
}

void beambreak() {
  beambroken = true;
}

float tilteq(float fa, float fdistance, float fheight) {
  tilteqr = tan(fa) * fdistance - 0.5 * 32.2 * pow((fdistance / 17.14), 2) * pow((1 / cos(fa)), 2) - fheight;
  return tilteqr;
}

void reset() {
  startscreen();
  beginpressed = false;
  confirmpressed = false;
  nofacerun = false;
  distgood = false;
  beambroken = false;
  aimed = false;
  abrun = false;
  rootfailure = false;
  firstroot = false;
  rootfound = false;
  secondroot = false;
  bpositive = false;
  anglegood = true;
  errorgood = false;
  maxangle = false;
  numgoldfish = 1;
  nofaceint = 10;
  Hservocommand = 90;
  iter = 0;
  e = 0;
  E = 0;
  eold = 0;
  de = 0;
  t = 0.1;
  a = 0;
  b = 0;
}
