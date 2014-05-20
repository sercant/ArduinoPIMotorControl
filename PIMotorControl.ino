int ledPin = 13; 
volatile byte rpmcount;
unsigned int rpm;
unsigned long timeold;
int fanPin = 5;
int speeda;
#include <LiquidCrystal.h>
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

//PI controller variables
double lastErr = 0, lastOutput = 255, kp = 0.9, ki = 3.6, dT = 0.5;

double compute(double setPoint, double actual) {
  double error = setPoint - actual;

  /* Compute PID Output */
  double output = kp * (error - lastErr) + ki * error * dT + lastOutput;
  output *= 0.17;
  // output += 90;
  // /* Max 255, Min -255 */
  if (output > 255) {
    output = 255;
  } 
  else if (output < -255) {
    output = -255;
  
  /* Remember some variables for next time */
  lastOutput = output;
  lastErr = error;
  }
  return output;
}

void rpm_fun()
{
  rpmcount++;
}

void setup()
{
  lcd.begin(16, 2);

  attachInterrupt(0, rpm_fun, FALLING);

  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  rpmcount = 0;
  rpm = 0;
  timeold = 0;

  pinMode(fanPin, OUTPUT);
  //Serial.begin(9600);
  //while (! Serial);
  //Serial.println("PWM cycle 0% to 100%");
}

void loop()
{
  delay(500);

  //speeda = 255 * Serial.parseInt();
  //speeda /= 100;
  //if (speeda >= 0 && speeda <= 255)
  //{

  //}
  detachInterrupt(0);
  rpm = ((float) 60 / 9) * 1000 / (millis() - timeold) * rpmcount;
  timeold = millis();
  rpmcount = 0;
  
  int pwm = (int) (90 + compute(1200, rpm));
  if (pwm > 255) {
    pwm = 255;
  } 
  else if (pwm < 0) {
    pwm = 0;
  }

  analogWrite(fanPin, pwm);

  lcd.clear();
  lcd.print("RPM=");
  lcd.print(rpm);
  lcd.setCursor(0, 2);
  lcd.print("Cycle= ");
  lcd.print((float)pwm / 255 * 100);
  attachInterrupt(0, rpm_fun, FALLING);
}



