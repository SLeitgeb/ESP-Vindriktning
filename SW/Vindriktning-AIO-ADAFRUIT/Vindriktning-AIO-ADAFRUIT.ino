/*
* Vzorovy kod od laskakit.cz pro LaskaKit ESP-VINDRIKTNING
* Kod podporuje cidla: SCD41 (CO2, teplota a vlhkost) nebo SHT40 (teplota a vlhkost)
* Kod podporuje posilani dat na: server io.adafruit.com nebo seriovy port (UART)
*
* LaskaKit ESP-VINDRIKTNING (https://www.laskakit.cz/laskakit-esp-vindriktning-esp-32-i2c/)
* LaskaKit SCD41 Senzor CO2, teploty a vlhkosti vzduchu (https://www.laskakit.cz/laskakit-scd41-senzor-co2--teploty-a-vlhkosti-vzduchu/)
* LaskaKit SHT40 Senzor teploty a vlhkosti vzduchu (https://www.laskakit.cz/laskakit-sht40-senzor-teploty-a-vlhkosti-vzduchu/)
*
* Vytvoreno (c) laskakit.cz 2022
* Upravil @cqeta1564
*
* Potrebne knihovny:
* https://github.com/adafruit/Adafruit_SHT4X //SHT40
* https://github.com/sparkfun/SparkFun_SCD4x_Arduino_Library //SCD41
* https://github.com/bertrik/pm1006 //PM1006
*/

#include "config.h"
// Interval mereni (zadavano v milisekundach)
#define INTERVAL_AL 500
#define INTERVAL_PM 30000
#define FAN_TIME_BEFORE_PM 10000
#define FAN_TIME_AFTER_PM 1000
#define FAN_TIME_OFF (INTERVAL_PM - FAN_TIME_BEFORE_PM - FAN_TIME_AFTER_PM)

// PM measure status
#define PM_STATUS_WAITING 0
#define PM_STATUS_FAN_BEFORE 1
#define PM_STATUS_FAN_AFTER 2

/* LaskaKit ESP-VINDRIKTNING - cidlo prasnosti PM1006 */
#define PIN_FAN 12 // spinani ventilatoru
#define RXD2 16 // UART - RX
#define TXD2 17 // UART - TX

#define PIN_BUZZER 2
#define PIN_AMBIENT_LIGHT 33
#define PIN_BUTTON 5


/* Nastaveni RGB LED */
#define BRIGHTNESS_DAY 55   // Day brightness
#define BRIGHTNESS_NIGHT 5  // Night brightness
#define BRIGHTNESS_DIFF (BRIGHTNESS_DAY - BRIGHTNESS_NIGHT)
#define DAY_MIN_AL 4000.0   // Maximum ambient light for day
#define DAY_MAX_AL 1000.0   // Maximum ambient light for day
#define AL_DIFF (DAY_MIN_AL - DAY_MAX_AL)
#define PIN_LED 25
#define PM_LED 0

volatile bool lights_on = true;
#ifdef button
volatile bool button_pressed = false;
unsigned long button_time = 0;
unsigned long last_button_time = 0;
#endif

#define ADA_URI "https://io.adafruit.com/api/v2/" IO_USERNAME
#define CO2_FEED  "co2"
#define PM25_FEED "pm-2-5"
#define TEMP_FEED "temp"
#define HUMI_FEED "humi"

// zapnout seriovy port (UART)
// #define uart

// zapnout server adafruit
// #define ada

// zapnout SCD41
// #define scd41

// #define bmp280

#ifdef bmp280
#include <Adafruit_BMP280.h>
Adafruit_BMP280 bmp;
#endif

// zapnout SHT40
//#define sht40

// zapnout OLED displej
// #define oled

// zapnout bzučák
// #define buzzer
/*------------------------ KONEC UPRAV -----------------------*/

#include <Wire.h>

/* ADAFRUIT */
#ifdef ada
#include <WiFi.h>
#include <HTTPClient.h>
#endif

/* SHT40 */
#ifdef sht40
#include "Adafruit_SHT4x.h"
#endif

/* OLED Dislpej */
#ifdef oled
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif

/* LaskaKit ESP-VINDRIKTNING s čidlem CO2/teploty/vlhkosti SCD41 */
#ifdef scd41
#include "SparkFun_SCD4x_Arduino_Library.h"
#define CO2_LED 2
#define WIFI_LED 1
#else
#define CO2_LED 1
#define WIFI_LED 2
#endif

/* LaskaKit ESP-VINDRIKTNING - cidlo prasnosti PM1006 */
#include "pm1006.h"

/* RGB adresovatelne LED */
#include <Adafruit_NeoPixel.h>

/* OLED nastaveni */
#ifdef oled
#define SCREEN_WIDTH  128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const unsigned char laskakit [] PROGMEM = {
	0x01, 0xf0, 0x00, 0x1f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x07, 0xfc, 0x00, 0x3f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 
	0x0f, 0xfe, 0x00, 0x7f, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 
	0x1e, 0x0f, 0x00, 0xf0, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xc0, 
	0x3c, 0x07, 0x80, 0xe0, 0x78, 0x00, 0x00, 0x0f, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 
	0x78, 0x03, 0xc0, 0x00, 0x3c, 0x00, 0x38, 0x0f, 0xc0, 0x03, 0x80, 0x00, 0x03, 0x80, 0x03, 0x00, 
	0x70, 0x01, 0xe0, 0x00, 0x1c, 0x00, 0x38, 0x00, 0x00, 0x03, 0x80, 0x00, 0x03, 0x80, 0x00, 0x00, 
	0xe0, 0x00, 0xf0, 0x00, 0x0e, 0x00, 0x38, 0x00, 0x00, 0x03, 0x80, 0x00, 0x03, 0x80, 0x00, 0x30, 
	0xe0, 0x00, 0x78, 0x00, 0x0e, 0x00, 0x38, 0x00, 0x00, 0x03, 0x80, 0x00, 0x03, 0x80, 0x00, 0x30, 
	0xe0, 0x00, 0x3c, 0x00, 0x0e, 0x00, 0x38, 0xfe, 0x0f, 0xe3, 0x8f, 0x1f, 0xc3, 0x8f, 0x1c, 0x7c, 
	0xe0, 0x00, 0x1e, 0x00, 0x0e, 0x00, 0x38, 0xff, 0x1f, 0xe3, 0x9e, 0x1f, 0xe3, 0x9e, 0x1c, 0x7c, 
	0x70, 0x00, 0x0f, 0x00, 0x1c, 0x00, 0x38, 0x03, 0x9c, 0x03, 0xbc, 0x00, 0xe3, 0xbc, 0x1c, 0x30, 
	0x78, 0x00, 0x07, 0x80, 0x3c, 0x00, 0x38, 0x03, 0x9e, 0x03, 0xf8, 0x00, 0xe3, 0xf8, 0x1c, 0x30, 
	0x3c, 0x00, 0x03, 0xc0, 0x78, 0x00, 0x38, 0xff, 0x8f, 0xe3, 0xf8, 0x1f, 0xe3, 0xf8, 0x1c, 0x30, 
	0x1e, 0x00, 0x01, 0xe0, 0xf0, 0x00, 0x39, 0xc3, 0x80, 0xf3, 0xfc, 0x38, 0xe3, 0xfc, 0x1c, 0x30, 
	0x0f, 0x00, 0x00, 0xff, 0xe0, 0x00, 0x39, 0xc3, 0x80, 0x73, 0x9e, 0x38, 0xe3, 0x9e, 0x1c, 0x30, 
	0x07, 0x80, 0x00, 0x7f, 0xc0, 0x00, 0x39, 0xff, 0x9f, 0xe3, 0x8f, 0x3f, 0xe3, 0x8f, 0x1c, 0x3e, 
	0x03, 0xc0, 0x00, 0x1f, 0x00, 0x00, 0x38, 0xff, 0x9f, 0xc3, 0x87, 0x1f, 0xe3, 0x87, 0x1c, 0x1e, 
	0x01, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x78, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x3c, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x1e, 0x00, 0x78, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x0f, 0x00, 0xf0, 0x00, 0x00, 0x20, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00, 0x08, 0x00, 0x00, 
	0x00, 0x07, 0x81, 0xe0, 0x00, 0x00, 0x3a, 0x9f, 0xba, 0xbb, 0x73, 0x66, 0x7e, 0xea, 0xed, 0xc0, 
	0x00, 0x03, 0xc3, 0xc0, 0x00, 0x00, 0x2a, 0x92, 0x8b, 0x2a, 0x62, 0x94, 0x4a, 0x2c, 0xa9, 0x80, 
	0x00, 0x01, 0xe7, 0x80, 0x00, 0x00, 0x29, 0x12, 0xaa, 0xa2, 0x12, 0x94, 0x4a, 0xaa, 0x88, 0x40, 
	0x00, 0x00, 0xff, 0x00, 0x00, 0x00, 0x39, 0x12, 0xb2, 0xba, 0x72, 0x64, 0x4a, 0xca, 0xe9, 0xc0, 
	0x00, 0x00, 0x7e, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x3c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
#endif


/* LaskaKit ESP-VINDRIKTNING s čidlem CO2/teploty/vlhkosti SCD41 */
#ifdef scd41
SCD4x SCD41;

#ifdef buzzer
#define BUZZER_FREQUENCY 4000
#define CO2_BUZZER_VOLUME 16
#define CO2_ALARM 1500 // ppm level to trigger alarm
#define CO2_ALARM_DELAY 900000 // 15 min
#define CO2_ALARM_LENGTH 2000
unsigned long last_co2_alarm = 1 << 31;
bool co2_alarm_on = false;

#define PM_BUZZER_VOLUME 128
#define PM_ALARM 850 // ppm level to trigger alarm
#define PM_ALARM_DELAY 300000 // 5 min
#define PM_ALARM_LENGTH 5000
unsigned long last_pm_alarm = 1 << 31;
bool pm_alarm_on = false;
#endif
#endif

/* LaskaKit ESP-VINDRIKTNING s čidlem teploty/vlhkosti SHT40 */
#ifdef sht40
Adafruit_SHT4x sht4 = Adafruit_SHT4x();
#endif

/* RGB adresovatelne LED */
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(3, PIN_LED, NEO_GRB + NEO_KHZ800);

int pm_status = PM_STATUS_WAITING;
unsigned long last_al_event = 0;
unsigned long last_pm_event = 0;
int next_pm_interval = FAN_TIME_BEFORE_PM;

/* LaskaKit ESP-VINDRIKTNING - cidlo prasnosti PM1006, nastaveni UART2 */
PM1006 * pm1006;
uint16_t pm2_5 = 65535;
int co2 = -1;
float temp = 9999.0;
int humidity = -1;

uint32_t COLOR_BLACK = pixels.Color(0, 0, 0);
uint32_t RED = pixels.Color(255, 0, 0);
uint32_t GREEN = pixels.Color(0, 255, 0);
uint32_t LT_GREEN = pixels.Color(128, 255, 0);
uint32_t YELLOW = pixels.Color(255, 255, 0);
uint32_t ORANGE = pixels.Color(255, 128, 0);
uint32_t BLUE = pixels.Color(0, 0, 255);

#ifdef button
void IRAM_ATTR toggle_lights()
{
  button_time = millis();
  if (button_time - last_button_time > 50)
  {
    lights_on = !lights_on;
    button_pressed = true;
    last_button_time = button_time;
  }
}
#endif

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  
  #ifdef buzzer
  if (!ledcAttach(PIN_BUZZER, BUZZER_FREQUENCY, 8)) Serial.println("Failed to configure buzzer!");
  #endif

  pinMode(PIN_FAN, OUTPUT); // Ventilator pro cidlo prasnosti PM1006
  pinMode(PIN_AMBIENT_LIGHT, INPUT);  // Ambient light
  analogReadResolution(12);

  #ifdef button
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), toggle_lights, RISING);
  #endif

  /*-------------- PM1006 - cidlo prasnosti ---------------*/
  Serial2.begin(PM1006::BIT_RATE, SERIAL_8N1, RXD2, TXD2); // cidlo prasnosti PM1006
  Serial.println("Initializing PM1006.");
  //                 UART     debug
  //                  |         |
  pm1006 = new PM1006(&Serial2, true);

  // PM1006 startup routine
  // we need to send it measurement command at least once to prevent the sensor from entering PWM mode
  // we also send it the command a few more times to actually start getting results back
  digitalWrite(PIN_FAN, HIGH);
  delay(500);
  int cycles = 5;
  while (cycles-- > 0)
  {
    if (pm1006->read_pm25(&pm2_5))
    {
      Serial.println("Obtained data from PM sensor.");
      #ifdef uart
      Serial.print("PM2.5: "); Serial.print(pm2_5); Serial.println(" ppm");
      #endif
    }
    else
    {
      pm2_5 = 65535;
      Serial.println("\nFailed to read PM data from sensor.");
    }
  }
  delay(100);
  digitalWrite(PIN_FAN, LOW);
  
  pixels.begin(); // WS2718
  set_LEDs();

  delay(10);
  
  /*-------------- RGB adresovatelne LED - zhasni --------------*/
  pixels.setPixelColor(PM_LED, COLOR_BLACK);
  pixels.setPixelColor(WIFI_LED, COLOR_BLACK);
  pixels.setPixelColor(CO2_LED, COLOR_BLACK);
  pixels.show();  // Zaktualizuje barvu

  /*-------------------- OLED Displej --------------------*/
  #ifdef oled
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  // Clear the buffer
  display.clearDisplay();

  display.drawBitmap(0, 0, laskakit, 128, 32, WHITE);
  display.display();
  #endif

  /*------------- SCD41 - CO2, teplota, vlhkost -----------*/
  #ifdef scd41
  // inicializace
  //             begin, autokalibrace
  //               |      |
  if (SCD41.begin(false, true) == false)
  {
    Serial.println("SCD41 nenalezen.");
    Serial.println("Zkontroluj propojeni.");
    while(1)
      ;
  }
 
  // prepnuti do low power modu
  if (SCD41.startLowPowerPeriodicMeasurement() == true)  
  {
    Serial.println("Low power mod povolen.");
  }
  #endif

  /*------------- SHT40 - teplota, vlhkost -----------*/
  #ifdef sht40
  if (! sht4.begin()) 
  {
    Serial.println("SHT4x not found");
    Serial.println("Check the connection");
    while (1) delay(1);
  }

  sht4.setPrecision(SHT4X_HIGH_PRECISION); // highest resolution
  sht4.setHeater(SHT4X_NO_HEATER); // no heater
  #endif

  /*------------- Wi-Fi -----------*/
  #ifdef ada
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.println("Pripojovani");
  int retry_counter = 10;
  while (WiFi.status() != WL_CONNECTED && retry_counter-- > 0)
  {
    pixels.setPixelColor(WIFI_LED, RED);
    pixels.show();
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED)
  {
    pixels.setPixelColor(WIFI_LED, GREEN);
    pixels.show();
    Serial.println();
    Serial.print("Pripojeno do site, IP adresa zarizeni: ");
    Serial.println(WiFi.localIP());
  }
  #endif

  light_sequence();
  next_pm_interval = millis() + FAN_TIME_OFF;
}

void light_sequence()
{
  for (uint16_t i = 0; i < 256; ++i)
  {
    delay(20);
    pixels.setPixelColor(PM_LED, pixels.ColorHSV((65535 - i) << 9));
    pixels.setPixelColor(WIFI_LED, pixels.ColorHSV(i << 8));
    pixels.setPixelColor(CO2_LED, pixels.ColorHSV(i << 9));
    pixels.show();
  }
  pixels.setPixelColor(PM_LED, COLOR_BLACK);
  pixels.setPixelColor(WIFI_LED, COLOR_BLACK);
  pixels.setPixelColor(CO2_LED, COLOR_BLACK);
  pixels.show();
}

void loop()
{
  unsigned long t_now = millis();
  if (t_now - last_pm_event >= next_pm_interval)
  {
    last_pm_event = t_now;
    process_pm_event();
  }

  #ifdef button
  if (button_pressed)
  {
    set_LEDs();
    button_pressed = false;
    Serial.println("Lights toggled!");
  }
  #endif

  t_now = millis();
  if (t_now - last_al_event >= INTERVAL_AL)
  {
    last_al_event = t_now;
    set_LEDs();
  }

  #ifdef buzzer
  t_now = millis();
  if (co2_alarm_on && ((t_now - last_co2_alarm) >= CO2_ALARM_LENGTH))
  {
    Serial.println("CO2 alarm off.");
    co2_alarm_on = false;
    ledcWrite(PIN_BUZZER, 0);
  }

  if (pm_alarm_on && ((t_now - last_pm_alarm) >= PM_ALARM_LENGTH))
  {
    Serial.println("PM alarm off.");
    pm_alarm_on = false;
    ledcWrite(PIN_BUZZER, 0);
  }
  #endif
}

void set_LEDs()
{
  #ifdef ada
  if (WiFi.status() == WL_CONNECTED)
  {
    pixels.setPixelColor(WIFI_LED, GREEN);
  }
  else
  {
    pixels.setPixelColor(WIFI_LED, RED);
  }
  #endif

  uint32_t pm_color = get_pm_color();
  pixels.setPixelColor(PM_LED, pm_color);

  #ifdef scd41
  pixels.setPixelColor(CO2_LED, get_co2_color());
  #else
  pixels.setPixelColor(CO2_LED, pm_color);
  #ifndef ada
  pixels.setPixelColor(WIFI_LED, pm_color);
  #endif
  #endif

  int al = analogRead(PIN_AMBIENT_LIGHT);
  int brightness = lights_on
    ? max(
      AL_DIFF - max(al - DAY_MAX_AL, 0.0),
      0.0
    ) / AL_DIFF * BRIGHTNESS_DIFF + BRIGHTNESS_NIGHT
    : 0;
  pixels.setBrightness(brightness);
  pixels.show();
}

bool measure_extra_data()
{
  /*------------- SCD41 - CO2, teplota, vlhkost -----------*/
  #ifdef scd41
  int limit = 10;
  while (!SCD41.readMeasurement() && limit-- > 0) // cekani na nova data (zhruba 30s)
  {
    delay(50);
  }

  if (limit == 0)
  {
    co2 = -1;
    temp = 9999.0;
    humidity = -1;
    return false;
  }

  co2 = SCD41.getCO2();
  temp = SCD41.getTemperature();
  humidity = SCD41.getHumidity();

  #ifdef buzzer
  if (
    co2 > CO2_ALARM
    && lights_on
    && ((millis() - last_co2_alarm) > CO2_ALARM_DELAY)
    && (analogRead(PIN_AMBIENT_LIGHT) < DAY_MIN_AL)
  )
  {
    Serial.println("CO2 alarm triggered!");
    ledcWrite(PIN_BUZZER, CO2_BUZZER_VOLUME);
    co2_alarm_on = true;
    last_co2_alarm = millis();
  }
  #endif

  // odeslani hodnot pres UART
  #ifdef uart
  Serial.print("Teplota: "); Serial.print(temp); Serial.println(" degC");
  Serial.print("Vlhkost: "); Serial.print(humidity); Serial.println("% rH");
  Serial.print("CO2: "); Serial.print(co2); Serial.println(" ppm");
  #endif
  #endif


  /*------------- SHT40 - teplota, vlhkost -----------*/
  #ifdef sht40
  sensors_event_t humidity_event, temp_event; // temperature and humidity variables
  sht4.getEvent(&humidity_event, &temp_event);

  temp = temp_event.temperature;
  humidity = humidity_event.relative_humidity;

  pixels.setPixelColor(WIFI_LED, get_temp_color());
  pixels.show();

  // odeslani hodnot pres UART
  #ifdef uart
  Serial.print("Teplota: ");
  Serial.print(temp);
  Serial.println(" degC");
  Serial.print("Vlhkost: ");
  Serial.print(humidity);
  #endif
  #endif

  return true;
}

void process_pm_event()
{
  if (pm_status == PM_STATUS_WAITING)
  {
    measure_extra_data();
    digitalWrite(PIN_FAN, HIGH);
    Serial.println("Fan ON");
    // schedule measurement
    pm_status = PM_STATUS_FAN_BEFORE;
    next_pm_interval = FAN_TIME_BEFORE_PM;
    if (millis() - last_pm_event > 5000) delay(5000);
  }
  else if (pm_status == PM_STATUS_FAN_BEFORE)
  {
    if (pm1006->read_pm25(&pm2_5))
    {
      Serial.println("Obtained data from PM sensor.");

      #ifdef uart
      Serial.print("PM2.5: "); Serial.print(pm2_5); Serial.println(" ppm");
      #endif

      #ifdef buzzer
      if (
        pm2_5 > PM_ALARM
        && pm2_5 != 65535
        && ((millis() - last_pm_alarm) > PM_ALARM_DELAY)
      )
      {
        Serial.println("PM alarm triggered!");
        ledcWrite(PIN_BUZZER, PM_BUZZER_VOLUME);
        pm_alarm_on = true;
        last_pm_alarm = millis();
      }
      #endif
    }
    else
    {
      pm2_5 = 65535;
      Serial.println("Failed to read PM data from sensor.");
    }
    #ifdef ada
    sendHTTPData();
    #endif
    // schedule fan off
    pm_status = PM_STATUS_FAN_AFTER;
    next_pm_interval = FAN_TIME_AFTER_PM;
  }
  else if (pm_status == PM_STATUS_FAN_AFTER)
  {
    digitalWrite(PIN_FAN, LOW);
    Serial.println("Fan OFF");
    // schedule fan on
    pm_status = PM_STATUS_WAITING;
    next_pm_interval = FAN_TIME_OFF;
  }


  /*------------- OLED Display -------------*/
  #ifdef oled
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text

  display.setCursor(5,8);             // Start at top-left corner
  display.println(String(pm2_5) + "mg/m3");

  #ifdef sht40
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text

  display.setCursor(69,5);             // Start at top-left corner
  display.println(String(temp) + "degC");  
  display.setCursor(85,21);             // Start at top-left corner
  display.println(String(humidity) + "%");
  display.setCursor(5,21);             // Start at top-left corner
  display.println(String(pm2_5) + "mg/m3");
  #endif

  #ifdef scd41
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text

  display.setCursor(5,8);             // Start at top-left corner
  display.println(String(co2));
  display.setCursor(92,8);             // Start at top-left corner
  display.println(String(pm2_5));

  display.setTextSize(1);
  display.setCursor(40,24);
  display.println("ppm");
  display.setCursor(92,24);
  display.println("mg/m3");
  #endif
  
  display.display();  
  delay(1);
  #endif
}

uint32_t get_co2_color()
{
  if (co2 < 0) return COLOR_BLACK;
  if (co2 < 700) return GREEN;
  if (co2 < 1000) return LT_GREEN;
  if (co2 < 1500) return YELLOW;
  if (co2 < 2000) return ORANGE;
  return RED;
}

uint32_t get_temp_color()
{
  if (temp < 20.0) return BLUE;
  if (temp < 23.0) return GREEN;
  if (temp < 9999) return RED;
  return COLOR_BLACK;
}

uint32_t get_pm_color()
{
  if (pm2_5 < 0) return COLOR_BLACK;
  if (pm2_5 < 25) return GREEN;
  if (pm2_5 < 40) return LT_GREEN;
  if (pm2_5 < 65) return YELLOW;
  if (pm2_5 < 100) return ORANGE;
  return RED;
}

String feed_value(String name, String value)
{
  return "{\"key\":\"" + name + "\",\"value\":" + value + "}";
}

#ifdef ada
void sendHTTPData()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Wi-Fi disconnected.");
    return;
  }
  
  String feed_values =
    pm2_5 != 65535 ? feed_value(PM25_FEED, String(pm2_5)) : "";
  #ifdef sht40
  if (humidity != -1) {
    if (feed_values != "")
      feed_values += ",";
    feed_values += feed_value(HUMI_FEED, String(humidity));
  }
  if (temp != 9999.0) {
    if (feed_values != "")
      feed_values += ",";
    feed_values += feed_value(TEMP_FEED, String(temp));
  }
  #endif
  #ifdef scd41
  if (co2 != -1) {
    if (feed_values != "")
      feed_values += ",";
    feed_values += feed_value(CO2_FEED, String(co2));
  }
  if (humidity != -1) {
    if (feed_values != "")
      feed_values += ",";
    feed_values += feed_value(HUMI_FEED, String(humidity));
  }
  if (temp != 9999.0) {
    if (feed_values != "")
      feed_values += ",";
    feed_values += feed_value(TEMP_FEED, String(temp));
  }
  #else
  #endif

  if (feed_values == "") return;
  String data = "{\"feeds\":[" + feed_values + "]}";

  HTTPClient http;
  if (http.begin(ADA_URI "/groups/vindriktning/data.json"))
  {
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-AIO-Key", IO_KEY);
    int httpResponseCode = http.POST(data);
    
    if (httpResponseCode > 0) 
    {
      Serial.print("HTTP ");
      Serial.println(httpResponseCode);
    }
    else 
    {
      Serial.print("Error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }
}

bool getAdaLightsOn()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Wi-Fi disconnected.");
    return true;
  }

  HTTPClient http;
  if (http.begin(ADA_URI "/feeds/vindriktning.lights-on/data/last?include=value"))
  {
    http.addHeader("X-AIO-Key", IO_KEY);
    int httpResponseCode = http.GET();
    
    if (httpResponseCode == 200)
    {
      Serial.print("Lights on: ");
      bool value = http.getString() == "{\"value\":\"ON\"}";
      Serial.println(value);
      http.end();
      return value;
    }
    else 
    {
      Serial.print("Lights on error code: ");
      Serial.println(httpResponseCode);
    }
    // Free resources
    http.end();
  }
  return true;
}
#endif

void reboot()
{
  Serial.println("Rebooting ESP...");

  // Blink red 10 times
  for (int i = 0; i < 10; i++) {
    pixels.setPixelColor(CO2_LED, RED);
    pixels.setPixelColor(WIFI_LED, RED);
    pixels.setPixelColor(PM_LED, RED);
    pixels.show();
    delay(200);
    pixels.setPixelColor(CO2_LED, COLOR_BLACK);
    pixels.setPixelColor(WIFI_LED, COLOR_BLACK);
    pixels.setPixelColor(PM_LED, COLOR_BLACK);
    pixels.show();
    delay(200);
  }

  // Reboot ESP
  ESP.restart();
}
