#include <Servo.h>
Servo servo1;
Servo servo2;

const int sampleWindow = 50; // Sample window width in mS (250 mS = 4Hz)
const double decayTime = .4;
const int counterStartVal = decayTime * (1000.0 / sampleWindow);

unsigned int knock0;
unsigned int knock1;
unsigned int knock2;
unsigned int knock3;

int counterLeft = counterStartVal;
int counterRight = counterStartVal;
int counterUp = counterStartVal;
int counterDown = counterStartVal;
double leftLoud = 0.0;
double rightLoud = 0.0;
double upLoud = 0.0;
double downLoud = 0.0;

bool leftStarted = false;
bool rightStarted = false;
bool upStarted = false;
bool downStarted = false;

const double ambientNoise = 0.1;


void setup()
{
  Serial.begin(9600);

  servo1.attach(9);
  servo2.attach(10);
}

void loop()
{
  unsigned long start = millis(); // Start of sample window
  double peakToPeak0 = 0.0;   // peak-to-peak level
  double peakToPeak1 = 0.0;
  double peakToPeak2 = 0.0;
  double peakToPeak3 = 0.0;

  unsigned int signalMax0 = 0;
  unsigned int signalMax1 = 0;
  unsigned int signalMax2 = 0;
  unsigned int signalMax3 = 0;

  unsigned int signalMin0 = 1024;
  unsigned int signalMin1 = 1024;
  unsigned int signalMin2 = 1024;
  unsigned int signalMin3 = 1024;


  // collect data for 250 miliseconds
  while (millis() - start < sampleWindow)
  {
    knock0 = analogRead(0);
    if (knock0 < 1024) { //This is the max of the 10-bit ADC so this loop will include all readings
      if (knock0 > signalMax0) {
        signalMax0 = knock0;  // save just the max levels
      } else if (knock0 < signalMin0) {
        signalMin0 = knock0;  // save just the min levels
      }
    }
    knock1 = analogRead(1);
    if (knock1 < 1024) { //This is the max of the 10-bit ADC so this loop will include all readings
      if (knock1 > signalMax1) {
        signalMax1 = knock1;  // save just the max levels
      } else if (knock1 < signalMin1) {
        signalMin1 = knock1;  // save just the min levels
      }
    }

    knock2 = analogRead(2);
    if (knock2 < 1024) {
      if (knock2 > signalMax2) {
        signalMax2 = knock2;
      } else if (knock2 < signalMin2) {
        signalMin2 = knock2;
      }
    }

    knock3 = analogRead(3);
    if (knock3 < 1024) {
      if (knock3 > signalMax3) {
        signalMax3 = knock3;
      } else if (knock3 < signalMin3) {
        signalMin3 = knock3;
      }
    }
  }

  peakToPeak0 = signalMax0 - signalMin0;  // max - min = peak-peak amplitude
  peakToPeak1 = signalMax1 - signalMin1;  // max - min = peak-peak amplitude
  peakToPeak2 = signalMax2 - signalMin2;
  peakToPeak3 = signalMax3 - signalMin3;

  // * 3.3 / 1024 / 2.16 is the same as just /670
  //  double volts0 = shave((peakToPeak0 * 3.3) / 1024 / 2.16, 0.1);
  //  double volts1 = shave((peakToPeak1 * 3.3) / 1024 / 2.16, 0.1);

  double volts0 = shave((peakToPeak0 / 670), 0.1);
  double volts1 = shave((peakToPeak1 / 670), 0.1);
  double volts2 = shave((peakToPeak2 / 670), 0.1);
  double volts3 = shave((peakToPeak3 / 670), 0.1);

  //  double volts0 = shave(peakToPeak0, 0.1)/2.83;
  //  double volts1 = shave(peakToPeak1, 0.1)/2.83;

  //  volts0 = shave(volts0);
  //  volts1 = shave(volts1);

  Serial.println(volts0);
  Serial.println(volts1);
  Serial.println(volts2);
  Serial.println(volts3);
  Serial.println("----");

  //  Serial.println(peakToPeak0);
  //  Serial.println(peakToPeak1);

  // Serial.println(getAngleFromTwoMicrophones(volts0, volts1));

  //Serial.println(volts0 - volts1);



  if (counterLeft == 0) {
    leftLoud = 0.0;
    counterLeft = counterStartVal;
    leftStarted = false;
  }
  if (counterRight == 0) {
    rightLoud = 0.0;
    counterRight = counterStartVal;
    rightStarted = false;
  }

  if (counterUp == 0) {
    upLoud = 0.0;
    counterUp = counterStartVal;
    upStarted = false;
  }
  if (counterDown == 0) {
    downLoud = 0.0;
    counterDown = counterStartVal;
    downStarted = false;
  }

  if (volts1 > ambientNoise || volts0 > ambientNoise) {

    double angle = getAngleFromTwoMicrophones(volts0, volts1);

    //Serial.println(angle);

    if ((volts1 > leftLoud) && (volts0 > rightLoud)) { //both increasing
      counterLeft = counterStartVal;
      counterRight = counterStartVal;
      leftStarted = true;
      rightStarted = true;
      leftLoud = volts1;
      rightLoud = volts0;
      rotateTo(servo1, angle, 10, 180);


    } else if (volts0 > rightLoud) { //right louder
      counterRight = counterStartVal;
      rightStarted = true;
      if (leftStarted) {
        counterLeft = counterLeft - 1;
      }
      rightLoud = volts0;
      rotateTo(servo1, angle, 10, 180);


    } else if (volts1 > leftLoud) { //left louder
      counterLeft = counterStartVal;
      leftStarted = true;
      if (rightStarted) {
        counterRight = counterRight - 1;
      }
      leftLoud = volts1;
      rotateTo(servo1, angle, 10, 180);

    }
  } else { //both under volume threshold
    if (leftStarted) {
      counterLeft = counterLeft - 1;
    }
    if (rightStarted) {
      counterRight = counterRight - 1;
    }
  }









  if (volts3 > ambientNoise || volts2 > ambientNoise) {

    double angle = getAngleFromTwoMicrophones(volts2, volts3);

    //Serial.println(angle);

//left (volts3) become up, right (volts 2) became down
    if ((volts3 > upLoud) && (volts2 > downLoud)) { //both increasing
      counterUp = counterStartVal;
      counterDown = counterStartVal;
      upStarted = true;
      downStarted = true;
      upLoud = volts3;
      downLoud = volts2;
      rotateTo(servo2, angle, 10, 170);


    } else if (volts2 > downLoud) { //down louder
      counterDown = counterStartVal;
      downStarted = true;
      if (upStarted) {
        counterUp = counterUp - 1;
      }
      upLoud = volts2;
      rotateTo(servo2, angle, 10, 170);


    } else if (volts3 > upLoud) { //up louder
      counterUp = counterStartVal;
      upStarted = true;
      if (downStarted) {
        counterDown = counterDown - 1;
      }
      upLoud = volts3;
      rotateTo(servo2, angle, 10, 170);

    }
  } else { //both under volume threshold
    if (upStarted) {
      counterUp = counterUp - 1;
    }
    if (downStarted) {
      counterDown = counterDown - 1;
    }
  }


}


double shave(double volt, double thresh) {
  if (volt < thresh) {
    return 0.0;
  }
  else {
    return volt;
  }


}

void rotateTo(Servo servo, int angle, int servoMin, int servoMax) {
  angle = min(max(servoMin, angle), servoMax);
  servo.write(angle);

}


double getAngleFromTwoMicrophones(double left, double right) {
  left = min(max(0.0, left), 1.0);
  right = min(max(0.0, right), 1.0);

  return (acos(right - left)) * 180 / 3.14;
}
