//BLE biblioteker
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//Accelerometer biblioteker
#include <Wire.h>                                                   // ACC
#include <SPI.h>                                                    // ACC
#include <SparkFunLSM9DS1.h>                                        //ACC
#include <math.h>                                                   //ACC
#include<TurboTrig.h>                                               // ACC
LSM9DS1 imu_1;                                                      //benytter "LSM9DS1 class" til at lave et objekt.
LSM9DS1 imu_2;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"  // The service we will advertise
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"  // The characteristic we will provide access to
#define LSM9DS1_AG_1 0x6B // definere adresse. SDO_AG is HIGH hexidecimal adresse 107 decimal
#define LSM9DS1_AG_2 0x6A // SDO_AG is LOW hexidecimal 106 Decimal
#define DECLINATION -8.58 // decline ned ad bakke, incline op ad bakke. 
#define FILTER_LENGTH 3 //Sætter filter længden til at være 4
#define Notch_length 40 //array til notchfilter (passer til 2000 Hz)

//Styringsvariabler 
int8_t ACC = 0;                                                        //ACC
volatile int16_t V_EMG;                                                // variabel til at lagre EMG
volatile int16_t H_EMG;
volatile int16_t hoejremyo;
volatile int16_t venstremyo;

//Filtervariabler
uint8_t value [5];    // unsigned int data type can hold integer values in the range of 0 to 4,294,967,295. / Array til at lagre data fra EMG1, EGM2 og accelerometer
volatile int16_t x_notch[Notch_length + 1] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Laver et array til filterene
volatile int16_t x_notch_1[Notch_length + 1] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // Laver et array til filterene

volatile int16_t x[FILTER_LENGTH + 1] = {0, 0, 0, 0};
volatile int16_t x_1[FILTER_LENGTH + 1] = {0, 0, 0, 0};

volatile int16_t y_org, y_filt, y_filt_1, y_filt_2, y_org_low, y_filt_low; // Sætter en int for hvert filter
volatile int16_t diff, diff_1, mavg, mavg_1;


//BLE variabler
BLEServer* pServer = NULL;                                         // Pointer to our server(BLE)
BLECharacteristic* pCharacteristic = NULL;                         // Pointer to our characteristic (BLE)

bool deviceConnected = false;                                      // Bestemmer startsværdien for deviceConnected til false
bool oldDeviceConnected = false;                                   // Bestemmer startsværdien for oldDeviceConnected til false
bool notifyServer = false;                                         // Bestemmer startsværdien for notifyServer til false


class MyServerCallbacks: public BLEServerCallbacks {               // Callback function for connect/disconnect events
    void onConnect(BLEServer* pServer) {                           // Function for a connect event
      deviceConnected = true;                                      // Signal the main loop that we just got connected
    }
    void onDisconnect(BLEServer* pServer) {                        // Function for a disconnect event
      deviceConnected = false;                                     // Signal the main loop that we just got disconnected
    }
};

//Accelerometer variabler
float angle_1 = 0;                                                 //ACC
float angle_2 = 0;                                                 //ACC
float angle_dg = 0;                                                //ACC
const float Pi = 3.141592653589793238462643383279502884197169399375105820974944592307816;

// Interrupt variabler
volatile int interruptCounter, LEDtaeller;                                     //ISR
hw_timer_t * timer = NULL;                                         //indikere starttid
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;              //indikere sluttid

////Funktioner

//Beregninger af vinkler
void calAngle()                                                    //laver funktion der omregner tyngdeacceleration i de forskellige akser til grader
{
  angle_1 = acos (imu_1.calcAccel(imu_1.az) / sqrt(sq(imu_1.calcAccel(imu_1.ax)) + sq(imu_1.calcAccel(imu_1.ay)) + sq(imu_1.calcAccel(imu_1.az))));
  angle_2 = acos (imu_2.calcAccel(imu_2.az) / sqrt(sq(imu_2.calcAccel(imu_2.ax)) + sq(imu_2.calcAccel(imu_2.ay)) + sq(imu_2.calcAccel(imu_2.az))));
  angle_1 = (angle_1 * 180) / Pi;                                  //omregner radianer til grader
  angle_2 = (angle_2 * 180) / Pi;
  if (imu_1.calcAccel(imu_1.ax) < 0)                               //Sørger for at der kan differentieres mellem negative og positive vinkler
  {
    angle_1 = angle_1 * (-1);
  }
  if (imu_2.calcAccel(imu_2.ax) < 0)
  {
    angle_2 = angle_2 * (-1);
  }

  angle_dg = angle_1 - angle_2;                                    //kompensere med reference værdi, således findes hovedet vinkel i forhold til kørestolen

  if (angle_dg > -20 && angle_dg < 20)
  {
    ACC = 2;
  }
  if (angle_dg < -20) //-1 hvis vinkel under -20
  {
    ACC = 1;
  }
  if (angle_dg > 20) // 1 hvis vinkelen er under 20
  {
    ACC = 3;
  }
}

//Interrupt funktion
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;          //tæller interrupts. her skal der indsætte kode
  hoejremyo = analogRead(39);
  y_filt = derivative(hoejremyo);
  H_EMG = rektificer_and_mavg(y_filt);

  venstremyo = analogRead(36);
  y_filt_1 = derivative_1(venstremyo);
  V_EMG = rektificer_and_mavg_1(y_filt_1);

  if (interruptCounter <= 120000) {      //1 min
    if (hoejremyo >= 3788) {
      digitalWrite(2, HIGH);              //grøn LED
      LEDtaeller = interruptCounter;
    }
    if (venstremyo >= 3788) {
      digitalWrite(4, HIGH);              //blå
      LEDtaeller = interruptCounter;
    }

    if (hoejremyo == 4095 || venstremyo == 4095) {
      digitalWrite(0, HIGH);              //rød
      LEDtaeller = interruptCounter;
    }

    if (LEDtaeller + 200 <= interruptCounter) {
      digitalWrite(0, LOW);
      digitalWrite(2, LOW);
      digitalWrite(4, LOW);
    }
  }

  if (interruptCounter == 120001) {
      digitalWrite(0, LOW);
      digitalWrite(2, LOW);
      digitalWrite(4, LOW);
  }

  portEXIT_CRITICAL_ISR(&timerMux);
}


//Filter Beregninger
volatile int16_t derivative(volatile int16_t value1)
{
  x_notch[0] =  value1;                                             // Read received sample and perform typecast
  x_notch[0] = x_notch[0] - 2048;                                  //fjerner offset
  diff = (x_notch[0] - x_notch[40]);                               //differentiere (notchfilter/kam-filter)
  for (volatile int i = Notch_length - 1; i >= 0; i--)                      // Roll x and y arrays in order to hold old sample inputs and outputs
  {
    x_notch[i + 1] = x_notch[i];                                   // denne forløkke flytter arrayet frem for hver value, i
  }
  return diff;
}

volatile int16_t derivative_1(volatile int16_t value2)
{
  x_notch_1[0] =  value2;                                           // Read received sample and perform typecast
  x_notch_1[0] = x_notch_1[0] - 2048;                              //fjerner offset
  diff_1 = (x_notch_1[0] - x_notch_1[40]);                           //differentiere (notchfilter/kam-filter)
  for (volatile int i = Notch_length - 1; i >= 0; i--)                      // Roll x and y arrays in order to hold old sample inputs and outputs
  {
    x_notch_1[i + 1] = x_notch_1[i];                               // denne forløkke flytter arrayet frem for hver value, i
  }
  return diff_1;
}

volatile int16_t rektificer_and_mavg(volatile int16_t value_1) {
  x[0] =  value_1;                                                 // Read received sample and perform typecast
  if (x[0] < 0) {                                                  //rektificere
    x[0] = x[0] * (-1);
  }

  mavg = mavg + (x[0] >> 6) - (mavg >> 8) - 1;                       // moving average filter uden brug af array ((mavg >> 10) ) = offset i bitniveauer

  return mavg;
}

volatile int16_t rektificer_and_mavg_1(volatile int16_t value_2) {
  x_1[0] =  value_2;                                               // Read received sample and perform typecast
  if (x_1[0] < 0) {                                                //rektificere
    x_1[0] = x_1[0] * (-1);
  }

  mavg_1 = mavg_1 + (x_1[0] >> 6) - (mavg_1 >> 8) - 1;                      // moving average filter uden brug af array

  return mavg_1;
}


void setup() {
  Serial.begin(115200);
  analogSetCycles(255); //ADC'ens kondensator lades helt op
  pinMode(39, INPUT); //eller A3
  pinMode(36, INPUT); //eller A0


  imu_1.settings.device.commInterface = IMU_MODE_I2C;               // Bestemmer I2C kommunikationsprotikol
  imu_2.settings.device.commInterface = IMU_MODE_I2C;               // Bestemmer I2C kommunikationsprotikol
  imu_1.settings.device.agAddress = LSM9DS1_AG_1;                   // Bestemmer Adresse i struct
  imu_2.settings.device.agAddress = LSM9DS1_AG_2;

  if (!imu_1.begin() || !imu_2.begin())                             //Hvis imu_1 og imu_2 ikke køres så printes følgende fejl meldinger
  {
    Serial.println("Failed to communicate with LSM9DS1.");
    while (1);
  }

  BLEDevice::init("ESP32");                                         // Create the BLE Device
  BLEDevice::setMTU(517);                                           // Set MTU to x bytes
  pServer = BLEDevice::createServer();                              // Create the BLE Server
  pServer->setCallbacks(new MyServerCallbacks());                   // Set callback function for connect/disconnect events
  BLEService *pService = pServer->createService(SERVICE_UUID);      // Create the BLE Service
  pCharacteristic = pService->createCharacteristic(                 // Create a BLE Characteristic
                      CHARACTERISTIC_UUID,                          // UUID of the Characteristic
                      BLECharacteristic::PROPERTY_READ   |          // Server can read the Characteristic
                      BLECharacteristic::PROPERTY_WRITE  |          // Server can write the Characteristic
                      BLECharacteristic::PROPERTY_NOTIFY            // Server can ask for notifications from the Characteristic
                    );
  pCharacteristic->addDescriptor(new BLE2902());                    // Create a BLE Descriptor  (Herfra fåes notifications)
  pService->start();                                                // Start the service (her initialiseres servicen)
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();       // get pointer to advertising class
  pAdvertising->addServiceUUID(SERVICE_UUID);                       // Add service UUID (giver servicen en bestemt UUID, så der kan kommunikeres med klienten)
  pAdvertising->setScanResponse(false);                             // We do not allow to send a scan response(Dette gøres da vi ikke er en klient)
  pAdvertising->setMinPreferred(0x0);                               // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();                                    // Start advertising (ved brug af start advertising fukntionen)
  Serial.println("Ventelse...");                                    // Print info to the terminal


  timer = timerBegin(0, 80, true);                                  //timer 0 (der er 4 ialt). prescaler på 80 -> 12,5 ns*80 = 1 mikro S, true betyder at vi tæller op
  timerAttachInterrupt(timer, &onTimer, true);                      // sætter variablen timer til funktionen onTimer, true betyder edge triggering (ikke level)
  timerAlarmWrite(timer, 500, true);                                // sætter timeren til at tælle til 10^6. -> 1 sek, true betyder enable autoreload (tæller forfra)
  timerAlarmEnable(timer);  // starter timer

  //Indbygget LED sættes lav
  pinMode(0, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  digitalWrite(0, LOW);
  digitalWrite(2, LOW);
  digitalWrite(4, LOW);
}


void loop() {
  if (imu_1.accelAvailable() )
  {
    imu_1.readAccel();
  }
  if ( imu_2.accelAvailable() )
  {
    imu_2.readAccel();
  }
  if (deviceConnected && !oldDeviceConnected) {                     // we have just got connected
    Serial.println("Connected to client");                          // Print info to the terminal
    delay(1000);                                                    // give the server a chance to get things ready
    oldDeviceConnected = deviceConnected;                           // Prevent a reentry
    notifyServer = true;                                            // Start notification
  }

  if (!deviceConnected && oldDeviceConnected) {                     // we have just got disconnected
    Serial.println("Disconnected from client");                     // Print info to the terminal
    delay(500);                                                     // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising();                                    // restart advertising
    Serial.println("Start advertising");                            // Print info to the terminal
    oldDeviceConnected = deviceConnected;                           // Prevent a reentry
    notifyServer = false;                                           // Stop notification
  }
  if (notifyServer && interruptCounter >= 200) {                    // laver tidsstyret timer med funktionen millis()                            // connected and ready to notify
    calAngle();


    value[0] = ACC;                                                 // den skal være lig med "output" da "output er et udtryk for en tærskelværdi over, mellem eller under en vinkel
    value[1] = V_EMG;
    value[2] = (V_EMG >> 8);
    value[3] = H_EMG;
    value[4] = (H_EMG >> 8);
    interruptCounter = 0;
    pCharacteristic->setValue((uint8_t*)&value, sizeof(value));     // set pCharacteristic to point to new value
    pCharacteristic->notify();                                      // send "notify changed"

Serial.println(ACC);
Serial.println(V_EMG);
Serial.println(H_EMG);
Serial.println(",");
  }
}
