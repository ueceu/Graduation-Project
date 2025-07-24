#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MAX31865.h>
#include <PID_v1_bc.h>

#define LCD_I2C_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2

// Butonlar
#define BTN_MENU 2
#define BTN_UP 3
#define BTN_DOWN 5
#define BTN_OK 4

// MAX31865 tanımı ve pinleri
Adafruit_MAX31865 thermo = Adafruit_MAX31865(10, 11, 12, 13);
#define RNOMINAL 100.0
#define RREF 430.0

LiquidCrystal_I2C lcd(LCD_I2C_ADDR, LCD_COLUMNS, LCD_ROWS);

const int controlPin = 6;  // Çıkış pini (On/Off kontrolü için)

// Menü Durumları
int menuState = 0;  // Ana menü: 0, PID: 10-15, On/Off: 20-22, SlowPWM: 30-31
int subMenuState = 0;
int currentMenu = -1;  // -1: Ana Menü, 10: PID, 20: On/Off, 30: SlowPWM

// **Global Değişkenler**
int setTemp = 45;
float Kp = 1.6, Ki = 0.2, Kd = 3.5;
int hist = 0;

void setup() {
  Serial.begin(115200);
  thermo.begin(MAX31865_2WIRE);
  pinMode(6, OUTPUT);
  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  pinMode(BTN_OK, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("ENS Temperature");
  lcd.setCursor(0, 1);
  lcd.print("  Controller");
}

void loop() {
  if (menuState == 0 && digitalRead(BTN_OK) == LOW) {
    delay(200);
    menuState = 1;
    subMenuState = 0;
    currentMenu = -1;
    showMainMenu();
  }

  if (menuState == 1) {
    if (digitalRead(BTN_DOWN) == LOW) {
      delay(200);
      if (subMenuState < 2) subMenuState++;
      showMainMenu();
    }

    if (digitalRead(BTN_UP) == LOW) {
      delay(200);
      if (subMenuState > 0) subMenuState--;
      showMainMenu();
    }

    if (digitalRead(BTN_MENU) == LOW) {
      delay(200);
      menuState = 0;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("ENS Temperature");
      lcd.setCursor(0, 1);
      lcd.print("  Controller");
    }

    if (digitalRead(BTN_OK) == LOW) {
      delay(200);
      currentMenu = subMenuState * 10 + 10;
      menuState = currentMenu;
      subMenuState = 0;
      showSubMenu();
    }
  }

  if (currentMenu == 10) handlePIDMenu();
  if (currentMenu == 20) handleOnOffMenu();
  if (currentMenu == 30) handleSlowPWMMenu();
}

void showMainMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  if (subMenuState == 0) lcd.print("PID Kontrol");
  else if (subMenuState == 1) lcd.print("ON/OFF Kontrol");
  else if (subMenuState == 2) lcd.print("SlowPWM Kontrol");
}

void showSubMenu() {
  lcd.clear();
  if (currentMenu == 10) lcd.print("PID Kontrol");
  else if (currentMenu == 20) lcd.print("ON/OFF Kontrol");
  else if (currentMenu == 30) lcd.print("SlowPWM Kontrol");
}

void handlePIDMenu() {
  switch (subMenuState) {
    case 0: updateInput("SetTemp", setTemp); break;
    case 1: updateInput("Kp", Kp); break;
    case 2: updateInput("Ki", Ki); break;
    case 3: updateInput("Kd", Kd); break;
    case 4: runPIDControl(); break;
  }
}

void handleOnOffMenu() {
  switch (subMenuState) {
    case 0: updateInput("SetTemp", setTemp); break;
    case 1: updateInput("Hist", hist); break;
    case 2: runOnOffControl(); break;
  }
}

void handleSlowPWMMenu() {
  switch (subMenuState) {
    case 0: updateInput("SetTemp", setTemp); break;
    case 1: runSlowPWMControl(); break;
  }
}

void updateInput(const char* label, int& value) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(label);
  lcd.print(" = ");
  lcd.print(value, DEC);

  lcd.setCursor(0, 1);
  lcd.print("OK: Save");

  while (true) {
    if (digitalRead(BTN_UP) == LOW) {
      delay(200);
      value += 1;
      updateDisplay(label, value);
    }

    if (digitalRead(BTN_DOWN) == LOW) {
      delay(200);
      value -= 1;
      updateDisplay(label, value);
    }

    if (digitalRead(BTN_OK) == LOW) {
      delay(200);
      subMenuState++;
      break;
    }

    if (digitalRead(BTN_MENU) == LOW) {
      delay(200);
      if (subMenuState > 0) {
        subMenuState--;
      } else {
        menuState = 1;
        currentMenu = -1;
        showMainMenu();
      }
      break;
    }
  }
}

void updateInput(const char* label, float& value) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(label);
  lcd.print(" = ");
  lcd.print(value, 2);

  lcd.setCursor(0, 1);
  lcd.print("OK: Save");

  while (true) {
    if (digitalRead(BTN_UP) == LOW) {
      delay(200);
      value += 0.10;
      updateDisplay(label, value);
    }

    if (digitalRead(BTN_DOWN) == LOW) {
      delay(200);
      value -= 0.10;
      updateDisplay(label, value);
    }

    if (digitalRead(BTN_OK) == LOW) {
      delay(200);
      subMenuState++;
      break;
    }

    if (digitalRead(BTN_MENU) == LOW) {
      delay(200);
      if (subMenuState > 0) {
        subMenuState--;
      } else {
        menuState = 1;
        currentMenu = -1;
        showMainMenu();
      }
      break;
    }
  }
}

template<typename T>
void updateDisplay(const char* label, T value) {
  lcd.setCursor(0, 0);
  lcd.print(label);
  lcd.print(" = ");
  lcd.print(value, 2);
}

void updateDisplay(const char* label, int value) {
  lcd.setCursor(0, 0);
  lcd.print(label);
  lcd.print(" = ");
  lcd.print(value, DEC);
}
void runPIDControl() {
  static unsigned long previousMillis = 0;
  const long interval = 1000;

  double input, output;
  double setpoint = setTemp;
  double error, lastError = 0, integral = 0;
  double dt = 1;

  while (true) {
    if (millis() - previousMillis >= interval) {
      previousMillis = millis();

      float temperature = thermo.temperature(RNOMINAL, RREF);
      input = temperature;
      error = setpoint - input;

      integral += error * dt;
      integral = constrain(integral, -50, 50);
      double derivative = (error - lastError) / dt;

      output = Kp * error + Ki * integral + Kd * derivative;
      output = constrain(output, 0, 255);

      if (abs(error) < 0.2 && output < 30) {
        output = 0;
      }

      analogWrite(controlPin, (int)output);
      lastError = error;

      double sc_output = output * 100 / 255;

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sicaklik: ");
      lcd.print(temperature, 1);

      lcd.setCursor(0, 1);
      lcd.print("PID: ");
      lcd.print((int)sc_output);

      Serial.print(temperature);
      Serial.print("\t");
      Serial.println((int)sc_output);
    }

    if (digitalRead(BTN_MENU) == LOW) {
      delay(200);
      digitalWrite(controlPin, LOW);
      menuState = 1;
      currentMenu = -1;
      showMainMenu();
      break;
    }
  }
}
void runOnOffControl() {
  static unsigned long previousMillis = 0;
  const long interval = 500;

  while (true) {
    if (millis() - previousMillis >= interval) {
      previousMillis = millis();

      float temperature = thermo.temperature(RNOMINAL, RREF);

      if (temperature <= (setTemp - hist)) {
        digitalWrite(controlPin, HIGH);
      } else if (temperature >= (setTemp + hist)) {
        digitalWrite(controlPin, LOW);
      }

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("TEMP: ");
      lcd.print(temperature, 1);

      lcd.setCursor(0, 1);
      lcd.print("RLY: ");
      lcd.print(digitalRead(controlPin) ? "OPEN" : "CLSD");

      Serial.print(temperature);
      Serial.print("\t");
      Serial.println(digitalRead(controlPin));
    }

    if (digitalRead(BTN_MENU) == LOW) {
      delay(200);
      digitalWrite(6, LOW);
      menuState = 1;
      currentMenu = -1;
      showMainMenu();
      break;
    }
  }
}
void SlowPWM(byte pin, signed long d, unsigned long T) {
  static unsigned long then = millis();
  d = constrain(d, 0, 100);

  if ((millis() - then) >= T - (T * d / 100)) {
    digitalWrite(pin, HIGH);
  }

  if ((millis() - then) >= T) {
    digitalWrite(pin, LOW);
    then = millis();
  }
}

void runSlowPWMControl() {
  static unsigned long previousMillis = 0;
  const long interval = 1000;
  static int pwmOutput = 0;

  while (true) {
    if (millis() - previousMillis >= interval) {
      previousMillis = millis();

      float temperature = thermo.temperature(RNOMINAL, RREF);

      if (temperature < setTemp) {
        pwmOutput = map(temperature, 0, setTemp, 0, 100);
      } else {
        pwmOutput = 0;
      }

      SlowPWM(6, pwmOutput, 1000);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Sicaklik: ");
      lcd.print(temperature, 1);

      lcd.setCursor(0, 1);
      lcd.print("PWM: ");
      lcd.print(temperature < setTemp ? 100 - pwmOutput : pwmOutput);

      Serial.print(temperature);
      Serial.print("\t");
      Serial.println(temperature < setTemp ? 100 - pwmOutput : pwmOutput);
    }

    if (digitalRead(BTN_MENU) == LOW) {
      delay(200);
      digitalWrite(6, LOW);
      menuState = 1;
      currentMenu = -1;
      showMainMenu();
      break;
    }
  }
}
