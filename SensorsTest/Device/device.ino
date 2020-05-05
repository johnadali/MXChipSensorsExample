#include "AZ3166WiFi.h"
#include "OledDisplay.h"
#include "Sensor.h"


#define LOOP_DELAY          1000

// Peripherals 
static DevI2C *ext_i2c;
static LSM6DSLSensor *accelgyroSensor;
static HTS221Sensor *tempSensor;
static LPS22HBSensor *pressureSensor;
static LIS2MDLSensor *magneticSensor;
static RGB_LED rgbLed;

// Indicate whether WiFi is ready
static bool hasWifi = false;

// Temperature sensor variables
static unsigned char tempSensorId;
static float humidity;
static float temperature;
static float pressure;

// Accelerometer, pedometer variables
static unsigned char accelGyroId;
static int stepCount;
static int xAxesData[3];
static int gAxesData[3];

// Magnetic variables
static unsigned char magSensorId;
static int axes[3];

static void InitWiFi()
{
  Screen.clean();
  Screen.print(2, "Connecting...");
  
  if (WiFi.begin() == WL_CONNECTED)
  {
    IPAddress ip = WiFi.localIP();
    Screen.print(1, ip.get_address());
    hasWifi = true;
    Screen.print(2, "Running... \r\n");
  }
  else
  {
    hasWifi = false;
    Screen.print(1, "No Wi-Fi\r\n ");
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
// Arduino sketch
void setup()
{
  Screen.init();
  Screen.print(1, "Initializing...");
  
  Screen.print(2, " > Serial");
  Serial.begin(115200);
  
  // Initialize the WiFi module
  Screen.print(3, " > WiFi");
  hasWifi = false;
  InitWiFi();
  if (!hasWifi)
  {
    return;
  }
  
  // Initialize LEDs
  Screen.print(3, " > LEDs");
  rgbLed.turnOff();
  
  // Initialize button
  Screen.print(3, " > Button");
  pinMode(USER_BUTTON_A, INPUT);
  pinMode(USER_BUTTON_B, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
  // Initialize the motion sensor
  Screen.print(3, " > Motion sensor     ");
  ext_i2c = new DevI2C(D14, D15);
  accelgyroSensor = new LSM6DSLSensor(*ext_i2c, D4, D5);
  accelgyroSensor->init(NULL); 
  accelgyroSensor->enableAccelerator();
  accelgyroSensor->enableGyroscope();
  accelgyroSensor->enablePedometer();
  accelgyroSensor->setPedometerThreshold(LSM6DSL_PEDOMETER_THRESHOLD_MID_LOW);

  stepCount = 0;

  // Initialize the temperature sensor
  Screen.print(3, " > Temperature sensor     ");
  tempSensor = new HTS221Sensor(*ext_i2c);
  tempSensor->init(NULL);

  // Initialize the pressure sensor
  Screen.print(3, " > Pressure sensor     ");
  pressureSensor = new LPS22HBSensor(*ext_i2c);
  pressureSensor->init(NULL);

  // Initialize the magnetic sensor
  Screen.print(3, " > Magnetic sensor     ");
  magneticSensor = new LIS2MDLSensor(*ext_i2c);
  magneticSensor->init(NULL);

  temperature = -1;
  humidity = -1;
  pressure = -1;

}

void loop()
{
  if (hasWifi)
  {
      Serial.println("Sensor Values");

      // Read temperature, humidity sensor
      tempSensor->enable();
      tempSensor->readId(&tempSensorId);
      Serial.printf("tempSensorId: %d\n", tempSensorId);

      tempSensor->getTemperature(&temperature);
      Serial.printf("Temperature: %f\n", temperature);

      tempSensor->getHumidity(&humidity);
      Serial.printf("Humidity: %f\n", humidity);
      tempSensor->disable();
      tempSensor->reset();

      accelgyroSensor->readId(&accelGyroId);
      Serial.printf("accelGyroId: %d\n", accelGyroId);

      // Read pedometer sensor
      accelgyroSensor->getStepCounter(&stepCount);
      Serial.printf("Pedometer Step Count: %d\n", stepCount);

      // Read accelerometer sensor
      accelgyroSensor->getXAxes(xAxesData);
      Serial.printf("Accelerometer X Axes: x=%d, y=%d, z=%d\n", xAxesData[0], xAxesData[1], xAxesData[2]);

      // Read gyroscope sensor
      accelgyroSensor->getGAxes(gAxesData);
      Serial.printf("Accelerometer G Axes: x=%d, y=%d, z=%d\n", gAxesData[0], gAxesData[1], gAxesData[2]);

      // Read pressure sensor
      pressureSensor->getPressure(&pressure);
      Serial.printf("Pressure=%f\n", pressure);

      // Read magnetic sensor
      magneticSensor->readId(&magSensorId);
      magneticSensor->getMAxes(axes);
      Serial.printf("magSensorId: %d\n", magSensorId);
      Serial.printf("Magnetometer Axes: x=%d, y=%d, z=%d\n", axes[0], axes[1], axes[2]);

      Serial.println(" ");
  }

  delay(1000);  // Every 1 second delay
}
