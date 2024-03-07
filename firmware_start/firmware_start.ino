// -------------------
// Includes
// -------------------

#include <Adafruit_NeoPixel.h>
#include "DHT.h"

// -------------------
// GPIO pins
// -------------------

#define     DHT11_PIN        0      // GPIO for DHT11 input data
#define     TACHO_PIN        0      // GPIO for TACH RPM inputsignal
#define     LEDDATA_PIN      0      // GPIO for LED data output
#define     FANPWM_PIN       0      // GPIO for FAN PWM outputsignal

// -------------------
// Define constants
// -------------------

#define     DHT_TYPE        DHT11   // Type of temperature sensor
#define     LED_COUNT       0       // Number of leds in the fan
#define     PWM_CH_FAN      1       // Fan on channel 1
#define     PWM_RES         0       // Resolution
#define     PWMFREQ         0       // Frequency of the PWM signal in Hz
#define     TACHOCYCLE      1000    // how often tacho speed shall be determined, in milliseconds
#define     INTERRUPS       0       // Number of interrupts ESP32 sees on tacho signal on a single fan rotation.

// -------------------
// Constants
// -------------------

const int MAX_DC = (int)(pow(2, PWM_RES) - 1);  // Max power of fan
const int limit1 = 30;  // Temperature limit 1
const int limit2 = 40;  // Temperature limit 2
const int limit3 = 50;  // Temperature limit 3

// -------------------
// Variables
// -------------------

Adafruit_NeoPixel fanled = Adafruit_NeoPixel(LED_COUNT, LEDDATA_PIN, NEO_GRB + NEO_KHZ800);
DHT dht(DHT11_PIN, DHT_TYPE);
static volatile int counter_rpm = 0;
int last_rpm = 0;
unsigned long lastTachoMeasurement = 0;

// -------------------
// Interupt methode to increase the rpm counter
// -------------------

void IRAM_ATTR rpm_fan() {
    counter_rpm++;
}

// -------------------
// Setup program
// -------------------

void setup() {
    // Initialize serialmonitor
    Serial.begin(115200);
    delay(2000);
    Serial.println("PC Fan Controller");
    // Initialize temperature sensor
    dht.begin();
    // Initialize LED
    fanled.begin();
    fanled.show();
    // Initialize FAN
    ledcSetup(PWM_CH_FAN, PWMFREQ, PWM_RES);
    ledcAttachPin(FANPWM_PIN, PWM_CH_FAN);  
    // Initialize TACHO
    pinMode(TACHO_PIN, INPUT);
    digitalWrite(TACHO_PIN, HIGH);
    attachInterrupt(digitalPinToInterrupt(TACHO_PIN), rpm_fan, FALLING);
    // kick start the FAN
    Serial.println("Kickstarting Fan...");
    ledcWrite(PWM_CH_FAN, MAX_DC);
    delay(2000);    
    Serial.println("Starting normal operation...");
}

// -------------------
// Loop program
// -------------------

void loop() {
    
    // Get temperature and show on serial monitor
    float t = dht.readTemperature();    
    Serial.print(F("Temperature: "));
    Serial.print(t);
    Serial.print(F("°C "));
    // Set fanspeed and show on serial monitor
    float duty = 0.25 ;
    if(t>limit1) { duty = 0.5 ; }
    if(t>limit2) { duty = 0.75 ; }
    if(t>limit3) { duty = 1; }    
    ledcWrite(PWM_CH_FAN, duty * MAX_DC);
    Serial.print(F("Fanspeed: "));
    Serial.print(duty*100);
    Serial.print(F("% "));    
    // Show RGB rainbow
    uint16_t i, j;
    for (j = 0; j < 256 * 5; j++)
    { 
        // Setting RGB led colors
        for (i = 0; i < fanled.numPixels(); i++) 
        {
            fanled.setPixelColor(i, Wheel(((i * 256 / fanled.numPixels()) + j) & 255));               
        }
        // Showing RGB colors
        fanled.show();      
        // Small delay
        delayClever(20);               
    }
    updateTacho();
}

// -------------------
// Wait a small delay
// -------------------

void delayClever(uint32_t milli)
{
    uint32_t stime = millis();

    while ((millis() - stime < milli));    
}

// -------------------
// Shift the positions of the leds
// -------------------

uint32_t Wheel(byte WheelPos) {
    WheelPos = 255 - WheelPos;
    if (WheelPos < 85) {
        return fanled.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    }
    if (WheelPos < 170) {
        WheelPos -= 85;
        return fanled.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    }
    WheelPos -= 170;
    return fanled.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}



// -------------------
// Update the tacho reading
// -------------------

void updateTacho(void) {
    // start of tacho measurement
    if ((unsigned long)(millis() - lastTachoMeasurement) >= TACHOCYCLE)
    { 
        // detach interrupt while calculating rpm
        detachInterrupt(digitalPinToInterrupt(TACHO_PIN)); 
        // calculate and show the rpm on the serial monitor
        last_rpm = counter_rpm * ((float)60 / (float)INTERRUPS) * ((float)1000 / (float)TACHOCYCLE);
        Serial.print(F("RPM: "));
        Serial.print(counter_rpm);
        Serial.println("\r\n");
        // reset counter
        counter_rpm = 0; 
        // store milliseconds when tacho was measured the last time
        lastTachoMeasurement = millis();
        // attach interrupt again
        attachInterrupt(digitalPinToInterrupt(TACHO_PIN), rpm_fan, FALLING);
    }
}
