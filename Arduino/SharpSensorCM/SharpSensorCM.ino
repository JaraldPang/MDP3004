#include <SharpIR.h>

#define TL 0
#define TM 2
#define TR 4

#define BRT 1
#define BRB 3
#define BLT 5

#define irTL A0
#define irTM A2
#define irTR A4

#define irBRT A1
#define irBRB A3
#define irBLT A5

// (pin , # readings before mean calculation, difference between 2 consecutive readings to be taken as valid, number)
SharpIR sensorTL(irTL, 15, 50, TL);  
SharpIR sensorTM(irTM, 15, 50, TM);
SharpIR sensorTR(irTR, 15, 50, TR);

SharpIR sensorBRT(irBRT, 15, 50, BRT);
SharpIR sensorBRB(irBRB, 15, 50, BRB);
SharpIR sensorBLT(irBLT, 15, 50, BLT);




void setup() {

  Serial.begin(115200);
  
  pinMode(irTL, INPUT);
  pinMode(irTM, INPUT);
  pinMode(irTR, INPUT);
  pinMode(irBRT, INPUT);
  pinMode(irBRB, INPUT);
  pinMode(irBLT, INPUT);

  digitalWrite(irTL, LOW);
  digitalWrite(irTM, LOW);
  digitalWrite(irTR, LOW);
  digitalWrite(irBRT, LOW);
  digitalWrite(irBRB, LOW);
  digitalWrite(irBLT, LOW);
 

}





void loop() {
  sendInfo('e');
}

void sendInfo(char c) {
  String execute = String("");

  if (c == 'g') {
    execute += String(c);
    Serial.println(execute);
  }
  else {
    if (c == 'e') {
      execute += String("");
    }
    else {
      execute += String(c);
    }
    String resultTL = String(final_MedianRead(irTL)) + String("\t");
    String resultTM = String(final_MedianRead(irTM)) + String("\t");
    String resultTR = String(final_MedianRead(irTR)) + String("\t");
    String resultBRT = String(final_MedianRead(irBRT)) + String("\t");
    String resultBRB = String(final_MedianRead(irBRB)) + String("\t");
    String resultBLT = String(final_MedianRead(irBLT));
    Serial.println(execute + resultTL + resultTM + resultTR + resultBRT + resultBRB + resultBLT);
  }
}

double final_MedianRead(int tpin) {
  double x[9];

  for (int i = 0; i < 9; i ++) {
    x[i] = distanceFinder(tpin);
  }

  insertionsort(x, 9);

  return x[4];
}

/*
   fs 7-15 1 block, 16-22 2 blocks
*/
double distanceFinder(int pin)
{
  double dis = 0.0;
  switch (pin)
  {
    case irTL:
      dis = sensorTL.distance();
      break;
    case irTM:
      dis = sensorTM.distance();
      break;
    case irTR:
      dis = sensorTR.distance();
      break;
    case irBRT:
      dis = sensorBRT.distance();
      break;
    case irBRB:
      dis = sensorBRB.distance();
      break;
    case irBLT:
      dis = sensorBLT.distance();
      break;
    default:
      break;
  }
  return dis;
}

void insertionsort(double array[], int length) {
  double temp;
  for (int i = 1; i < length; i++) {
    for (int j = i; j > 0; j--) {
      if (array[j] < array[j - 1])
      {
        temp = array[j];
        array[j] = array[j - 1];
        array[j - 1] = temp;
      }
      else
        break;
    }
  }
}
