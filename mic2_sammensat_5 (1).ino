//BLE//
#include "BLEDevice.h"
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");         // The remote service we wish to connect to.
static BLEUUID    charUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");         // The characteristic of the remote service
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;                      // Pointer to the remote characteristic
static BLEAdvertisedDevice* myDevice;                                      // Pointer to the remote device

//TID//
volatile uint32_t startTime = 0; // start time
volatile uint32_t currentTime = 0; // current time
unsigned int tidsstyring = 0;
unsigned int tid;

//Data FRA MIC_1//
int8_t acc = 0;
int16_t V_EMG = 0;
int16_t H_EMG = 0;

//GENSTART//
int counter = 0;

//KALIBRERING//
#include <M5Stack.h>
int array_EMG_v[40];
int index_v;
int array_EMG_h[40];
int index_h;
bool pressed = false;
bool kalibrering = false;
const int pin_kali = 38;
int max_v_EMG;
int max_h_EMG;

//STYRING//
//Hastigheder -1 km/t = 2, 0 km/t = 12, 1 km/t = 22, 3 km/t = 42, 5 km/t = 62
const int8_t hastighed[] = {2, 12, 22, 42, 62};
//Drejningskonstant
const float hjul_distance = 17;
const double F[] = {0.00094247 * hjul_distance, 0.00094247 * hjul_distance, 0.00094247 * hjul_distance, 0.00073513 * hjul_distance, 0.00052150 * hjul_distance}; //tidligere: {0.5724, 0.5724, 0.5724, 0.4464, 0.3168}
int16_t drej_v = 0, drej_h = 0;

//SIKKERHED//
//Accelerometeret i M5Stack
#include "utility/MPU9250.h"
#include <math.h>
MPU9250 IMU;
float angle_x = 0;
float angle_z = 0;
const float Pi = 3.1415926;
float x_kal = 0, y_kal = 0, z_kal = 0;
bool acc_kalibrering = true;
bool sikkerhedVal = 0;
//Afstandsmaaleren
long duration;  //tidsmåling af respons fra ultralydssensor
float distance; //Angiver distancen målt af ultralydssensoren
float distance_m;
const int trigPin = 26;
const int echoPin = 36;
//Hastighedsvariabler
int8_t hast = 1, v = 0;
float ny_hast;
//Accelerometer fra MIC1
int8_t acc_klar = 1, acc_fortsaet = 0, acceleration = 0;
//Hastighed for hvert hjul
int8_t v_hjul = 0, h_hjul = 0;
//Tid bundet til accelerationen
unsigned int startTidCase = 0;
//Nødbrems
const int pin_stop = 37;
bool pressed_2 = false;

//NXT KOMMUNIKATION//
#define MOTOR1 (0b01000000) // samme som 0x40 i hex , som er det samme som (1<<6)
#define MOTOR2 (0b10000000) // samme som 0x80 i hex, som er det samme som (2 <<6)
#define RXD2 16
#define TXD2 17
#define SET_MOTOR_POWER(y,x)((y) & (~(63)) | ((x) & (63))) // For at sikre at der ikke anvendes mere en de 3 nederste bit
unsigned char a = MOTOR1;
unsigned char b = MOTOR2;

//BLE FUNKTION
static void notifyCallback( BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) // callback til notify i ser
{
  acc = pData[0]; //Modtaget accelerometer data
  V_EMG = pData[1] + (pData[2] << 8); // Modtaget EMG data. Gemmes i EMG_v fra kalibrering
  H_EMG = pData[3] + (pData[4] << 8); // Modtaget EMG data. Gemmes i EMG_h fra kalibrering
}

//BLE CLASS
class MyClientCallback : public BLEClientCallbacks {  // Callback function for connect/disconnect events
    void onConnect(BLEClient* pclient) {              // Function for a connect event
      Serial.println("\n\rConnect event");            // Print info to the terminal
    }
    void onDisconnect(BLEClient* pclient) {          // Function for a disconnect event
      connected = false;                             // Signal the main loop that we just got disconnected
      doScan = true;                                 // Make the main loop do a rescan
      Serial.println("\n\rDisconnect event");        // Print info to the terminal
    }
};

//BLE CLASS
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {      // Called for each advertising BLE server
    void onResult(BLEAdvertisedDevice advertisedDevice) {                     // Check if the advertising BLE server have the right serviceUUID
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

        BLEDevice::getScan()->stop();                                   // The right serviceUUID is found, so stop scanning
        myDevice = new BLEAdvertisedDevice(advertisedDevice);           // Create a new device instance
        doConnect = true;                                               // Signal the main loop to connect to the server
        doScan = false;                                                 // Signal the main loop not to do a rescan
      }
    }
};

//BLE FUNKTION
static void scanCompleteCallback(BLEScanResults scanResults) {          // Callback invoked when scanning has completed.
  printf("Scan complete!\n");                                           // Print info to the terminal
  printf("We found %d devices\n", scanResults.getCount());              // Print number of found devices to the terminal
  doScan = true;                                                        // Make the main loop do a rescan
}

//BLE VARIABEL
bool connectToServer() {
  BLEClient*  pClient  = BLEDevice::createClient();                     // Create BLE client
  Serial.println(" - Created client");                                  // Print info to the terminal
  pClient->setClientCallbacks(new MyClientCallback());                  // Set callback function for connect/disconnect events
  pClient->connect(myDevice);                                           // Connect to the remote BLE Server.
  Serial.println(" - Connected to server");                             // Print info to the terminal
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);  // Obtain a reference to the service we are after in the remote BLE server.
  if (pRemoteService == nullptr) {                                      // A null pointer is returned if the service does not exist
    Serial.print("Failed to find our service UUID: ");                  // Print info to the terminal
    pClient->disconnect();                                              // Disconnect the client
    return false;                                                       // Return "false" to indicate failure
  }
  else
  {
    Serial.print(" - Found our service: ");                             // Print info to the terminal
    Serial.println(serviceUUID.toString().c_str());                     // Print serviceUUID to the terminal
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);  // Obtain a reference to the characteristic in the service of the remote BLE server.
  if (pRemoteCharacteristic == nullptr) {                               // A null pointer is returned if the characteristic does not exist
    Serial.print("Failed to find our characteristic UUID: ");           // Print info to the terminal
    Serial.println(charUUID.toString().c_str());                        // Print characteristicUUID to the terminal
    pClient->disconnect();                                              // Disconnect the client
    return false;                                                       // Return "false" to indicate failure
  }
  else
  {
    Serial.println(" - Found our characteristic");                      // Print info to the terminal
  }

  if (pRemoteCharacteristic->canNotify()) // kan kalibreringen er udført// Check if the characteristic allows notification
    pRemoteCharacteristic->registerForNotify(notifyCallback);           // Enable notification
  connected = true;                                                     // Signal the main loop that we are connected
  doScan = false;                                                       // Signal the main loop not to do a rescan
  return true;                                                          // Return "true" to indicate success
}

//KALIBRERING FUNKTION
void kalibrering_startet() { //Kalibrering
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextSize(4);
  M5.Lcd.setCursor(20, 20); M5.Lcd.print("KNAP ER");
  M5.Lcd.setCursor(20, 60); M5.Lcd.print("TRYKKET");
  M5.Lcd.setCursor(20, 120); M5.Lcd.print("KALIBRERER");
  M5.Lcd.setCursor(20, 160); M5.Lcd.print("I 8 SEK");
  Serial.println("EMG_v: ");
  for (index_v = 0; index_v <= 40; index_v++) { //Array med 40 pladser fyldes op med EMG data
    delay(100); //Samplerate paa 10 Hz
    array_EMG_v[index_v] = V_EMG; //Erstat analogRead med EMG data funktion
    Serial.println(array_EMG_v[index_v]); //Printer array

  }
  Serial.println("EMG_h: "); //Array for hoejre side
  for (index_h = 0; index_h <= 40; index_h++) {
    delay(100);
    array_EMG_h[index_h] = H_EMG;
    Serial.println(array_EMG_h[index_h]);

  }
  if (array_EMG_v[40] > 1) { //Hvis plads 40 er stoerre end 1 findes den maksimale vaerdi i arrayet
    array_EMG_v[index_v];
    max_v_EMG = array_EMG_v[0];
    for (int i; i < 40; i++) {
      if (array_EMG_v[i] > max_v_EMG) {
        max_v_EMG = array_EMG_v[i];  //Den maksimale værdi gemmes i max_v_EMG
      }
    }
    Serial.println("max_v: "); //Max vaerdi printes til Monitor
    Serial.println(max_v_EMG);
  }
  

  if (array_EMG_h[40] > 1) { //Max vaerdi for hoejre side
    array_EMG_h[index_h];
    max_h_EMG = array_EMG_h[0];
    for (int j; j < 40; j++) {
      if (array_EMG_h[j] > max_h_EMG) {
        max_h_EMG = array_EMG_h[j];
      }
    }
    Serial.println("max_h: ");
    Serial.println(max_h_EMG);

    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setCursor(20, 20);
    M5.Lcd.println("KALIBRERING");
    M5.Lcd.setCursor(20, 60);
    M5.Lcd.println("AFSLUTTET");
    delay(2000);
  }
}

//KALIBRERING FUNKTION
void IRAM_ATTR kalibrering_isr() { //Interrupt. Aktiverer kalibrering ved tryk paa knap B
  pressed = true;
  detachInterrupt(pin_kali);
}

//Sikkerhed - nødstop ///NY!!!!
void noedstop_isr() {
  pressed_2 = true;
  //detachInterrupt(pin_stop);
}

//STYRING FUNKTION
void afstandsmaaler() {  //Beregner distancen til objekter
  // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH, 10000);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Sikre at at ultralydssenderen ikke udtaler sig om noget den ikke ved noget om.
  // Ultralydssenderen kan detektere objekter op til en afstand på 400 cm
  if (distance > 127) {
    distance = 127;
  }
  if (distance < 0) { //Sikrer at distancen ikke kan være negativ
    distance = 0;
  }
  if (distance == 0) { // Når afstandsmåleren ikke kan læse er distance 0. For ikke at stoppe programmet skal denne i stedet være 400.
    distance = 127;
  }
}

//STYRING FUNKTION
void tilt_maaler() { //Indbygget accelerometer. Tiltfunktion
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU.readAccelData(IMU.accelCount);
    IMU.getAres();

    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes - x_kal; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes - y_kal; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes + z_kal; // - accelBias[2];

    angle_x = acos ((IMU.ax) / (sqrt(sq(IMU.ax) + sq(IMU.ay) + sq(IMU.az))));
    angle_x = (angle_x * 180) / Pi;  //omregner radianer til grader

    angle_z = acos ((IMU.az) / (sqrt(sq(IMU.ax) + sq(IMU.ay) + sq(IMU.az))));
    angle_z = (angle_z * 180) / Pi;  //omregner radianer til grader

    if ((angle_x < 78) && (angle_z > 15)) {
      hast = 1;
      v_hjul = hastighed[hast];
      h_hjul = hastighed[hast];
      sikkerhedVal = 1;

    }
    if ((angle_x > 100) && (angle_z > 15)) {
      hast = 1;
      v_hjul = hastighed[hast];
      h_hjul = hastighed[hast];
      sikkerhedVal = 1;
    }

    if ((angle_z < 15)) {
      sikkerhedVal = 0;
    }

    if (acc_kalibrering) { //Kalibrering for accelerometeret
      x_kal = IMU.ax;
      y_kal = IMU.ay;
      z_kal = 1 - IMU.az;

      acc_kalibrering = false;
    }

  }
}

//STYRING FUNKTION
void sikkerhed() {
  tilt_maaler();

  if (sikkerhedVal == 0) {
    afstandsmaaler();
    if (distance <= 31 && hast > 1) {           //skal skiftes
      v_hjul = hastighed[hast]; //v_hjul er hastigheden på venstre hjul
      h_hjul = hastighed[hast]; //h_hjul er hastigheden på højre hjul
      distance_m = distance / 100;
      if (distance_m < 0.10) {
        distance_m = 0;
      }

      ny_hast = (-0.6723 * (distance_m * distance_m) + (6.2359 * distance_m)-0.61687);          // gamle: 4 * (log((distance_m) + 0.3354) + 1.09243);

      if (ny_hast < 0) {
      ny_hast = 0;
    }
    
      ny_hast = (ny_hast * 40.65) + 12;

      if (ny_hast <= h_hjul) {
        v_hjul = ny_hast;
        h_hjul = ny_hast;
        sikkerhedVal = 1;
        drej_v = 0;
        drej_h = 0;
      }
      else {
        sikkerhedVal = 0;
      }
    }
  }
}

//STYRING FUNKTION
void accAnalyse() {
  //herefter sendes til forskellige cases afhængig af accelerometerets værdi
  //hvis acc_klar=0 så betyder det at vi er imellem hastighedsniveauer.
  //derfor sendes til samme case som før.
  //acc= et eller andet via bluetooth
  if ((startTidCase + 1000) > millis()) {
    switch (acc_fortsaet) {
      case 1:
        acceleration = 1;             //deccelerere
        break;
      case 3:
        acceleration = 3;             //accelerere
      default:
        break;
    }
  }

  if ((startTidCase + 1000) <= millis()) {
    switch (acc) {                         //bliver acc at til 1,2 og 3??
      case 1:                                //deccelerer
        acceleration = 1;
        hast = hast - 1;
        startTidCase = millis();
        break;
      case 2:                                 //konstant hastighed
        acceleration = 2;
        break;
      case 3:                                 //accelerer
        acceleration = 3;
        hast = hast + 1;
        startTidCase = millis();
        break;
      default:
        break;
    }
    if (hast < 0) {
      hast = 0;
    }
    if (hast > 4) {
      hast = 4;
    }
  }
}

//STYRING FUNKTION !!!!Ændre drej værdier!!!!
void drejAnalyse() {
  if ((V_EMG > 0.8 * max_v_EMG) && (H_EMG > 0.8 * max_h_EMG)) { //819 = 20% af 4095. Her drejes ikke
    drej_h = 0;
    drej_v = 0;
    hast = 1;
    while ((tid + 1000) > millis()) {
      //goer ingenting
    }
  }

  if ((V_EMG < 0.2 * max_v_EMG) && (H_EMG < 0.2 * max_h_EMG)) {
    drej_v = 0;
    drej_h = 0;
  }

  if ((V_EMG > 0.2 * max_v_EMG) && (H_EMG > 0.2 * max_h_EMG)) {
    drej_v = 0;
    drej_h = 0;
  }

  if ((H_EMG >= 0.2 * max_h_EMG) && (V_EMG < 0.2 * max_v_EMG)) { //drejes til højre.
    drej_h = ((H_EMG - 0.2 * max_h_EMG) / (max_h_EMG / 100)) *  F[hast] * 40.65;     // F er en drejningskonstant
    drej_v = 0;
  }

  if ((V_EMG >= 0.2 * max_v_EMG) && (H_EMG < 0.2 * max_h_EMG)) { //drejes til venstre.
    drej_v = ((V_EMG - 0.2 * max_v_EMG) / (max_v_EMG / 100)) * F[hast] * 40.65;     // F er en drejningskonstant og 10 den faktor der ganges op med i forhold til output 0-63.
    drej_h = 0;
  }

  //følgende if-statements gør at vi ikke kan dreje for meget.
  if (hast < 3) {     //Beregning: 0.00094247*hjul_distance (svare til en % kontraktion) * 30 (maksimal muskelkontraktion) * 40.65 (faktor) = 19.54
    if (drej_h > 19) {
      drej_h = 19;
    }
    if (drej_v > 19) {
      drej_v = 19;
    }
  }
  if (hast == 3) {     //Beregning: 0.00073513*hjul_distance (svare til en % kontraktion) * 30 (maksimal muskelkontraktion) * 40.65 (faktor) = 15.24
    if (drej_h > 15) {
      drej_h = 15;
    }
    if (drej_v > 15) {
      drej_v = 15;
    }
  }

  if (hast == 4) {            //Beregning: 0.00052150*hjul_distance (svare til en % kontraktion) * 30 (maksimal muskelkontraktion) * 40.65 (faktor) = 10.81
    if (drej_h > 10) {
      drej_h = 10;
    }
    if (drej_v > 10) {
      drej_v = 10;
    }
  }
}

//STYRING FUNKTION
void styring() {     //Her skal beskrives hvad der skal ske i de forskellige cases
  switch (acceleration) {
    case 1: //deceleration
      if (hast <= 1) {
        v_hjul = (drej_h + hastighed[hast]);
        h_hjul = (drej_v + hastighed[hast]);
      }
      if (hast >= 2) {
        v_hjul = (hastighed[hast] - drej_v);
        h_hjul = (hastighed[hast] - drej_h);
      }
      acc_fortsaet = 1;
      break;

    case 2: //drejer kun.
      if (hast <= 1) {
        v_hjul = (drej_h + hastighed[hast]);
        h_hjul = (drej_v + hastighed[hast]);
      }
      if (hast >= 2) {
        v_hjul = (hastighed[hast] - drej_v);
        h_hjul = (hastighed[hast] - drej_h);
      }
      acc_fortsaet = 2;
      break;

    case 3: //accelerere
      if (hast <= 1) {
        v_hjul = (drej_h + hastighed[hast]);
        h_hjul = (drej_v + hastighed[hast]);
      }
      if (hast >= 2) {
        v_hjul = (hastighed[hast] - drej_v);
        h_hjul = (hastighed[hast] - drej_h);
        break;
      }
      acc_fortsaet = 3;
      break;

    default:
      break;
  }
}

//NXT styring
void NXT_styring() {
  a = SET_MOTOR_POWER(a, h_hjul) ;    //Indsæt i stedet for 5 styrings-værdien
  b = SET_MOTOR_POWER(b, v_hjul) ;

  Serial2.write(a);
  Serial2.write(b);
}

void setup() {                                                   ////////////SETUP///////////////////
  Serial.begin(115200);
  M5.begin();
  Wire.begin();

  //Display
  M5.begin(true, false, true);
  M5.Lcd.clear(BLACK);
  M5.Lcd.setTextColor(WHITE);

  //Styring
  IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias);
  IMU.initMPU9250();
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);  // Sets the echoPin as an Input
  IMU.initAK8963(IMU.magCalibration);
  //tid = millis();

  //Tid
  tidsstyring = millis();

  //BLE
  Serial.println("Starting Arduino BLE Client application...");         // Print info to the terminal
  BLEDevice::init("");                                                  // Initialize the BLE environment (a client has no name)
  BLEDevice::setMTU(517);                                               // Set MTU to 517 bytes
  BLEScan* pBLEScan = BLEDevice::getScan();                             // Retrieve the Scan object that we use for scanning
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks()); // Set the callbacks to be invoked
  pBLEScan->setInterval(1349);                                          // Set the interval between scans (in ms)
  pBLEScan->setWindow(449);                                             // Set the window to actively scan in each interval (in ms)
  pBLEScan->setActiveScan(false);                                       // We perform a passive scan (no wish for a scan response)
  pBLEScan->start(10, scanCompleteCallback, false);                     // Start a scan for 10 seconds
  printf("Now scanning in the background ... scanCompleteCallback() will be called when done.\n");    // Print info to the terminal

  //Opstart tekst
  M5.Lcd.setTextSize(4);
  M5.Lcd.setCursor(20, 20);
  M5.Lcd.println("gruppe4403");
  delay(2000);

  //Kalibrering
  pinMode(pin_kali, INPUT);
  attachInterrupt(pin_kali, kalibrering_isr, FALLING);

  // Nødbrems
  pinMode(pin_stop, INPUT);
  attachInterrupt(pin_stop, noedstop_isr, FALLING);

  //NXT setup
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);

        drej_v = 0;
        drej_h = 0;
        hast=1;
        v_hjul=12;
        h_hjul=12;

        NXT_styring();
        
}

void loop() {                                                   ////////////LOOP///////////////////
  //NOT CONNECTED
  if (connected == false) {
    M5.Lcd.clear(BLACK);
    M5.Lcd.setTextSize(4);
    M5.Lcd.setCursor(20, 20);
    M5.Lcd.println("SYSTEMET");
    M5.Lcd.setCursor(20, 60);
    M5.Lcd.println("STARTES");
    M5.Lcd.setCursor(20, 150);
    M5.Lcd.println("forbinder...");
    delay(1000);
  }
  //CONNECTED prevent reentry & NOT CONNECTED
  if (doConnect == true) { //
    if (connectToServer()) {                                            // Call connectToServer
      Serial.println("We are now connected to the BLE Server.");        // Print info to the terminal
      doConnect = false;                                                // Prevent a reentry
      doScan = false;                                                   // Prevent a rescan
    } else {
      Serial.println("Ingen kommunikation, vi gør ikke mere.");         // Print info to the terminal
    }
  }
  //CONNECTED
  if (connected && tidsstyring + 100 <= millis()) // hvis vi er connected, så gør ting i loopet
  {
    tidsstyring = millis();

    if (kalibrering == false) { //Print på skærm inden kalibrering
      //Print til display
      M5.Lcd.clear(BLACK);
      M5.Lcd.setTextSize(4);
      M5.Lcd.setCursor(20, 20);
      M5.Lcd.println("KALIBRERING");
      M5.Lcd.setCursor(20, 60);
      M5.Lcd.println("STARTES");
      M5.Lcd.setCursor(20, 110);
      M5.Lcd.println("TRYK PAA");
      M5.Lcd.setCursor(20, 149);
      M5.Lcd.println("KNAP");
      M5.Lcd.setCursor(150, 220);
      M5.Lcd.setTextSize(2);
      M5.Lcd.println("OK");
    }

    if (pressed) {  //Kalibrering startes ved tryk på knap B, Interrupt
      pressed = false;
      kalibrering = true;
      kalibrering_startet();
    }

    if (kalibrering == true) { //Kalibreringen er afsluttet

      //Hastighedsniveauer på display
      M5.Lcd.clear(BLACK);
      M5.Lcd.setTextSize(4);
      M5.Lcd.setTextColor(WHITE);
      M5.Lcd.setCursor(10, 10); M5.Lcd.println("HAST_NIVEAU");
      M5.Lcd.setCursor(10, 80); M5.Lcd.println("v_hjul");
      M5.Lcd.setCursor(200, 80); M5.Lcd.println(v_hjul - 12);
      M5.Lcd.setCursor(10, 140); M5.Lcd.println("h_hjul");
      M5.Lcd.setCursor(200, 140); M5.Lcd.println(h_hjul - 12);
      M5.Lcd.setTextColor(RED);
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(235, 220); M5.Lcd.println("STOP");


      sikkerhed();
      if (sikkerhedVal == 0) {
        accAnalyse();
        drejAnalyse();
        styring();
      }
      
      if (pressed_2) { //Nødbrems ved tryk på knap C, Interrupt
        pressed_2 = false;
        drej_v = 0;
        drej_h = 0;
        hast=1;
        v_hjul=12;
        h_hjul=12;
        M5.Lcd.clear(BLACK);
        M5.Lcd.setTextColor(WHITE);
        M5.Lcd.setTextSize(4);
        M5.Lcd.setCursor(0, 20); M5.Lcd.println(">>NOEDBREMS<<");
         M5.Lcd.setCursor(20, 100);
      M5.Lcd.println("GENSTART");
      M5.Lcd.setCursor(20, 140);
      M5.Lcd.println("SYSTEMET");
         M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(20, 200);
      M5.Lcd.println("TRYK PAA ROED KNAP");
   //         NXT_styring();
//        while(1){
//        }
      }   
         
      NXT_styring();

    }
  }

  //Ingen forbindels/RESCAN
  else if (doScan) {                                                    // Do a rescan
    Serial.println("We are now doing a rescan.");
    BLEDevice::getScan()->start(10, scanCompleteCallback, false);       // Start a scan for 10 seconds
    counter ++;
    if (counter > 3) {
      M5.Lcd.clear(BLACK);
      M5.Lcd.setTextSize(4);
      M5.Lcd.setCursor(20, 20);
      M5.Lcd.println("GENSTART");
      M5.Lcd.setCursor(20, 60);
      M5.Lcd.println("SYSTEMET");
      M5.Lcd.setTextSize(2);
      M5.Lcd.setCursor(20, 200);
      M5.Lcd.println("TRYK PAA ROED KNAP");
      while (1) {
      }
    }
  }

}
