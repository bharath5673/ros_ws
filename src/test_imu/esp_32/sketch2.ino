#include <WiFi.h>
#include <WebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// Replace with your network credentials
//const char* ssid = "STARLINK v2";
const char* ssid = "get of my LAN";
const char* password = "P@$$w0rD";
//onboard led
const int onboardLED = 5;  


// Variables to store acceleration and gyroscope data
float ax, ay, az, gx, gy, gz;

WebServer server(80);
Adafruit_MPU6050 mpu;



int vSpeed = 50;
int i = 0;


void setup() {
  // Start serial communication
  Serial.begin(115200);

  // Connect to Wi-Fi network
  WiFi.begin(ssid, password);
  Serial.println("");
  
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  // Start the web server
  server.on("/", handleRoot);
  server.begin();
  Serial.println("Web server started");





  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }




}

void loop() {

  // Check if Wi-Fi connection is still active
  checkWifiConnection();

  // Read MPU6050 data
  readMPU6050();

  // Serial.print("ax: "); Serial.print(ax);
  // Serial.print(" ay: "); Serial.print(ay);
  // Serial.print(" az: "); Serial.print(az);
  // Serial.print(" gx: "); Serial.print(gx);
  // Serial.print(" gy: "); Serial.print(gy);
  // Serial.print(" gz: "); Serial.println(gz);


  // Handle incoming client requests
  server.handleClient();
}


// Handler for the root URL
void handleRoot() {
  String html = "<html><head><meta http-equiv='refresh' content='0.2'></head><body><h1 style='text-align:center;'></h1>";
  html += "<table style='margin:auto;'>";
  html += "<tr><td colspan='3' style='text-align:center;'><b>Accelerometer Values: <span id='accelerometerValues'>[" + String(ax) + "," + String(ay) + "," + String(az) + "]</span></b></td></tr>";
  html += "<tr><td colspan='3' style='text-align:center;'><b>Gyrometer Values: <span id='gyrometerValues'>[" + String(gx) + "," + String(gy) + "," + String(gz) + "]</span></td></b></tr>";
  html += "</table>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}


void BlinkLed() {
  digitalWrite(onboardLED, HIGH);
  delay(75);
  digitalWrite(onboardLED, LOW);
}


void checkWifiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    for (int i = 0; i < 3; i++) {
      digitalWrite(onboardLED, HIGH); // turn on the LED
      delay(50);
      digitalWrite(onboardLED, LOW); // turn off the LED
      delay(50);
    }
    digitalWrite(onboardLED, LOW); // turn off the LED
    delay(1200);          
  } else {
    digitalWrite(onboardLED, LOW); // turn off the LED
  }
}


// Function to read accelerometer and gyroscope data from MPU6050
void readMPU6050() {
  sensors_event_t a, g, temp;
  
  mpu.getEvent(&a, &g, &temp);
  
  // Print the accelerometer data
  Serial.print("Acceleration (m/s^2): ");
  Serial.print(a.acceleration.x);
  Serial.print(", ");
  Serial.print(a.acceleration.y);
  Serial.print(", ");
  Serial.print(a.acceleration.z);
  
  // Print the gyroscope data
  Serial.print(" | Gyroscope (rad/s): ");
  Serial.print(g.gyro.x);
  Serial.print(", ");
  Serial.print(g.gyro.y);
  Serial.print(", ");
  Serial.print(g.gyro.z);


  // Update global variables with sensor data
  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;

  Serial.println();
}

