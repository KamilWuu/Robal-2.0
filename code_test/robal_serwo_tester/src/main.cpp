#include <Wire.h>
#include <PCA9685.h>

// Inicjalizacja dwóch obiektów PCA9685 z różnymi adresami I2C
PCA9685 pwmController1(0x40); // Adres dla pierwszego PCA9685
PCA9685 pwmController2(0x41); // Adres dla drugiego PCA9685

#define SERVOMIN  500 // Minimalna szerokość impulsu (w jednostkach PWM)
#define SERVOMAX  2500 // Maksymalna szerokość impulsu (w jednostkach PWM)
#define SERVO_FREQ 50 // Częstotliwość PWM dla serwomechanizmów (typowo 50 Hz)

bool state = 0;

void setServoAngle(PCA9685 &pwmController, uint8_t channel, float angle) {
  // Ograniczanie kąta do zakresu 0-270°
  if (angle < 0) angle = 0;
  if (angle > 270) angle = 270;

  // Konwersja kąta na jednostki PWM
  uint16_t pulseLength = map(angle, 0, 270, SERVOMIN, SERVOMAX);

  // Ustawienie PWM dla wybranego kanału
  pwmController.setPWM(channel, 0, pulseLength);
}

void setup() {
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("PCA9685_LIB_VERSION: ");
  Serial.println(PCA9685_LIB_VERSION);
  Serial.println();

  Wire.begin();
  
  pwmController1.begin();
  pwmController2.begin();

  // Zakładamy, że częstotliwość jest ustawiana w inny sposób
  // Jeśli biblioteka nie obsługuje bezpośrednio ustawiania częstotliwości, dostosuj kod lub sprawdź dokumentację biblioteki
  
  // Inicjalizacja wszystkich kanałów na obu PCA9685 na wartość minimalną
  /*for (uint8_t i = 0; i < 16; i++) {
    pwmController1.setPWM(i, 0, SERVOMIN);
    pwmController2.setPWM(i, 0, SERVOMIN);
  }*/

  setServoAngle(pwmController1, 0, 116);
  setServoAngle(pwmController1, 1, 116);
  setServoAngle(pwmController1, 2, 116);
  delay(500);
  /*delay(2000);

  setServoAngle(pwmController1, 0, 150);
  setServoAngle(pwmController1, 1, 116);
  setServoAngle(pwmController1, 2, 116);
  delay(2000);

  setServoAngle(pwmController1, 0, 116);
  setServoAngle(pwmController1, 1, 116);
  setServoAngle(pwmController1, 2, 116);
  delay(2000);*/

}



void loop() {
  // Testowanie serwomechanizmów na obu PCA9685

  /*// Pierwszy PCA9685
  for (float angle = 0; angle <= 230; angle += 1) {
    setServoAngle(pwmController1, 0, angle);
    setServoAngle(pwmController2, 1, angle);
    delay(15);  // Opóźnienie dla płynnego ruchu
  }
  delay(500);

  for (float angle = 230; angle >= 0; angle -= 1) {
    setServoAngle(pwmController1, 0, angle);
    setServoAngle(pwmController2, 1, angle);
    delay(15);  // Opóźnienie dla płynnego ruchu
  }*/
  
  
  /*setServoAngle(pwmController1, 0, 0);
  delay(2000);

  setServoAngle(pwmController1, 0, 270);
  delay(2000);*/

 
  /*setServoAngle(pwmController1, 0, 0);

  delay(1000);

  setServoAngle(pwmController1, 0, 115);

  delay(5000);

  setServoAngle(pwmController1, 0, 230);

  delay(1000);*/
  
  /*setServoAngle(pwmController1, 0, 100);
  setServoAngle(pwmController1, 1, 100);
  setServoAngle(pwmController1, 2, 100);
  delay(1000);*/

  setServoAngle(pwmController1, 5, 170);
  setServoAngle(pwmController1, 6, 60);
  setServoAngle(pwmController1, 7, 220);
  delay(2000);

  

  

  setServoAngle(pwmController1, 0, 60);
  setServoAngle(pwmController1, 1, 170);
  setServoAngle(pwmController1, 2, 30);
  delay(2000);

  setServoAngle(pwmController1, 5, 116);
  setServoAngle(pwmController1, 6, 100);
  setServoAngle(pwmController1, 7, 180);
  delay(2000);

  setServoAngle(pwmController1, 0, 116);
  setServoAngle(pwmController1, 1, 140);
  setServoAngle(pwmController1, 2, 50);
  delay(2000);


  

  /*setServoAngle(pwmController1, 0, 116);
  setServoAngle(pwmController1, 1, 116);
  setServoAngle(pwmController1, 2, 116);
  delay(1000);*/


  

}
