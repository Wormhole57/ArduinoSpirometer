
#include <LiquidCrystal.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

#define BTN (9)   //digital button

// For BMP280 (SPI interface)
#define SCK  (13) //clock
#define MISO (6)  //SDO -> data from arduino to sensor
#define MOSI (7)  //SDI -> data from sensor to arduino
#define CS1  (10) //CSB -> pin for chip selection
#define CS2  (8)

// For LCD screen
#define D4  (5)   //data pins
#define D5  (4)
#define D6  (3)
#define D7  (2)
#define EN  (11)  //enable writing on memory (register)
#define RS  (12)  //integrated memory (register) cell select

LiquidCrystal lcd(RS, EN, D4, D5, D6, D7);
Adafruit_BMP280 bmp1(CS1, MOSI, MISO, SCK);
Adafruit_BMP280 bmp2(CS2, MOSI, MISO, SCK);

// Constants
const float pi = atan(1)*4;
const float dt = 0.001, dt_ms = 1.0; // Time delay [s],[ms]
const int   rows=2, cols=16;         // LCD display dimensions
const float rho = 1.1839;            // Density of air at 25° [kg/m3]  // 1.225 at 15°
const float R1 = 0.01;               // Radius inlet [m]
const float R2 = 0.025;              // Radius outlet [m]
const float S1 = pow(R1, 2) * pi;    // Surface area inlet [m2]
const float S2 = pow(R2, 2) * pi;    // Surface area outlet [m2]
const float beta = R1 / R2;          // Ratio of pipe radius
const float Cd = 0.9858 - 0.196 * pow(beta, 4.5);

// Variables
float p1 = 0, p2 = 0, pOffset;      // Pressures from sensors [Pa]
float pDelta = 0;                   // Pressure delta
float volFlow = 0, volFlowPrev = 0; // Volumetric flow rate calculated from pressure [L/s]
float volume = 0;                   // Integral of flow rate over time [L]
float fev1 = 0, ratio = 0;          // Pulmonary function indicators
unsigned long tPrev = 0, t = 0, tInit = 0;     // Time instants [us]
bool done = 0, isRunning = 0, resetScreen = 0; // Flags

// Initialization
void setup() {
  // LCD
  lcd.begin(cols, rows);
  lcd.print("WAIT PLEASE...");

  // Serial comunication
  Serial.begin(115200);
  while ( !Serial ) delay(100);

  // Button
  pinMode(BTN, INPUT);

  // Sensors
  unsigned status;
  status = bmp1.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor #1, check wiring or "
                      "try a different address!"));
    Serial.println("SensorID was: 0x"); Serial.println(bmp1.sensorID(),16);
    while (1) delay(10);
  }
  bmp1.setSampling(Adafruit_BMP280::MODE_NORMAL,
                   Adafruit_BMP280::SAMPLING_NONE,
                   Adafruit_BMP280::SAMPLING_X1,
                   Adafruit_BMP280::FILTER_OFF,
                   Adafruit_BMP280::STANDBY_MS_1);
  status = bmp2.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    Serial.println(F("Could not find a valid BMP280 sensor #2, check wiring or "
                      "try a different address!"));
    Serial.println("SensorID was: 0x"); Serial.println(bmp2.sensorID(),16);
    while (1) delay(10);
  }
  bmp2.setSampling(Adafruit_BMP280::MODE_NORMAL,
                   Adafruit_BMP280::SAMPLING_NONE,
                   Adafruit_BMP280::SAMPLING_X1,
                   Adafruit_BMP280::FILTER_OFF,
                   Adafruit_BMP280::STANDBY_MS_1);
  
  // Calculate difference between the 2 sensors
  for (int i = 0; i < 5000; i++) {
    p1 = bmp1.readPressure();
    p2 = bmp2.readPressure();
    pOffset += p2 - p1;
    delay(1);
  }
  pOffset /= 5000;

  // End initialization
  lcd.setCursor(0,0);
  lcd.print(" PRESS AND HOLD ");
  lcd.setCursor(0,1);
  lcd.print("    TO START!   ");
}

// Infinite Loop
void loop() {
  // Check if button is pressed
  if (digitalRead(BTN) == HIGH) {
    
    // Check if time is less than 6 sec
    if (t - tInit <= 6e6) {

      // Manage time flow
      if (tInit == 0) {
        tInit = micros();
        t = tInit;
      }
      tPrev = t;
      t = micros();

      // Change LCD text at start
      if (not(isRunning)) {
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("FVC=");
        lcd.setCursor(0,1);
        lcd.print("FEV1=");
      }
      isRunning = 1;

      // Fluid dynamic calculus
      p1 = bmp1.readPressure();
      p2 = bmp2.readPressure();
      pDelta = p2 - p1 - pOffset;
      if (pDelta < 0) pDelta = 0;
      volFlowPrev = volFlow;
      volFlow = 1e3 * Cd * S1 * S2 * sqrt( 2 * pDelta / (rho * (pow(S2,2) - pow(S1,2))) ) - 3;
      if (volFlow < 0) volFlow = 0;
      
      // Integration over time with trapezoidal rule
      volume += ((volFlow + volFlowPrev)/2) * (t - tPrev)/1.0e6;

      // Save FEV1
      if ((t - tInit >= 1.0e6) && (fev1 == 0)) {
        fev1 = volume;
        lcd.setCursor(5,1);
        lcd.print(fev1);
      }

      // Print current volume on LCD
      lcd.setCursor(4,0);
      lcd.print(volume);

      // Serial Output
      Serial.print((t - tInit) / 1.0e6);
      Serial.print(',');
      Serial.print(volume);
      Serial.print(',');
      Serial.print(volFlow);
      Serial.print(',');
      Serial.print(p2);
      Serial.print(',');
      Serial.print(p1);
      Serial.print(',');
      Serial.print(pDelta);
      Serial.println();

      // Minimum delay between cicles
      delay(1);

    // When test is finished
    } else {
      if (not(done)) {
        
        // Calculate RATIO
        ratio = 100 * fev1 / volume;

        // Final Serial Output
        Serial.println("EOF");
        Serial.println(fev1);
        Serial.println(volume);
        Serial.println(ratio);

        // Flags management
        done = 1;
        isRunning = 0;
      }
    }
    resetScreen = 1;

  // When BTN is not pressed
  } else {

    // Wait if a previous test has finished
    if (done) {
      delay(5000);
    }

    // Reset LCD
    if (resetScreen) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(" PRESS AND HOLD ");
      lcd.setCursor(0,1);
      lcd.print("    TO START!   ");
      resetScreen = 0;
    }

    // Reset variables
    volume = 0;
    fev1 = 0;
    tInit = 0;
    tPrev = 0;
    t = 0;

    // Flags management
    done = 0;
    isRunning = 0;
  }
}
