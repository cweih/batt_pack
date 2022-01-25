#include <Arduino.h>
#include <CAN.h>
#include <ShiftRegister74HC595.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ########### Defines #######################

// ++++ Für die Schieberegister ++++
#define NUMBER_OF_SHIFT_REGISTERS (2) // Anzahl der 8bit Shift Register die hintereinander geschaltet sind
#define SHIFT_REGISTER_DATA_PIN_NO (3) // GPIO Pin Nummer über den die Daten für das Register eingestellt werden
#define SHIFT_REGISTER_CLOCK_PIN_NO (1) // GPIO Pin Nummer über den die Pulse zum Übernehmen der Daten aus SHIFT_REGISTER_DATA_PIN_NO übertragen werden 
#define SHIFT_REGISTER_OUTPUT_PIN_NO (0) // GPIO Pin Nummer für den Latch Pin. Wenn hier ein Puls zu sehen ist werden die Daten aus dem Register in den Output übernommen

// ++++ Für die Temperatursensoren ++++
#define ONE_WIRE_BUS (2) // Daten werden über diesen Pin gesendet und empfangen (PIN 2 entspricht D4)
#define TEMP_SENS_ADDR_0 {0x28, 0xDC, 0x91, 0x82, 0x31, 0x21, 0x03, 0x91} // One Wire Bus Adresse von Sensor 0
#define TEMP_SENS_ADDR_1 {0x28, 0xFB, 0x90, 0x9D, 0x31, 0x21, 0x03, 0xCD} // One Wire Bus Adresse von Sensor 1
#define TEMP_SENS_ADDR_2 {0x28, 0xA7, 0x82, 0x60, 0x31, 0x21, 0x03, 0x8D} // One Wire Bus Adresse von Sensor 2

// ++++ Für die Strommessungen
#define CURR_SENS_ABSOLUTE_MEASUREMENT_RANGE (10) // [A] Strommessspanne von -5 bis 5 Ampere macht 10A absolut
#define CURR_SENS_ADC_BIT_RESOLUTION (1024) // Wertebereich am ADC von 0 bis 1024
#define CURR_SENS_MEASUREMENT_RANGE_OFFSET (-5) // [A] Messwertbereich geht nicht von 0 bis 10 sondern von -5 bis 5, muss also um -5 verschoben werden

//############ Globale Variablen ###############
// Variablen für das Relais Board
int schalterzustand = 1;

// ++++ Variablen für das Shift Register ++++
uint8_t ouput_register_1 = 0u;
ShiftRegister74HC595<NUMBER_OF_SHIFT_REGISTERS> shift_register_obj;

// ++++ Für die Temperatursensoren ++++
OneWire one_wire(ONE_WIRE_BUS); // Erzeuge ein oneWire Objekt um mit einem oneWire Gerät zu kommunizieren
DallasTemperature temp_sensors(&one_wire); //Erzeuge ein DallasTemperature Objekt mit Verweis auf das zu nutzende oneWire Objekt
int device_count = 0;
float temp_in_grad;
DeviceAddress Thermometer;
uint8_t temp_sens_0_addr[8] = TEMP_SENS_ADDR_0;
uint8_t temp_sens_1_addr[8] = TEMP_SENS_ADDR_1;
uint8_t temp_sens_2_addr[8] = TEMP_SENS_ADDR_2;


//############# Funktionsprototypen ############
void printAddress(DeviceAddress deviceAddress);
void printTemperature(DeviceAddress deviceAddress);



//########### MAIN ####################################################
//#####################################################################
void setup() {
  // temp_sensors.begin();	// Start up the library
  // Serial.begin(9600);
  // // locate devices on the bus
  // Serial.print("Suche Temperatursensoren...");
  // Serial.print("Gefunden ");
  // device_count = temp_sensors.getDeviceCount();
  // Serial.print(device_count, DEC);
  // Serial.println(" Temperatursensoren.");
  // Serial.println("");

  // start serial port
  Serial.begin(9600);

  // Start up the library
  temp_sensors.begin();

  // ACHTUNG: RX und TX Pin werden als GPIOs umdefiniert!
  //shift_register_obj.begin(SHIFT_REGISTER_DATA_PIN_NO, SHIFT_REGISTER_CLOCK_PIN_NO, SHIFT_REGISTER_OUTPUT_PIN_NO);

  // pinMode(D1, OUTPUT);
  // digitalWrite(D1, schalterzustand);
  // Serial.begin(9600);
  // while (!Serial);

  // Serial.println("CAN Sender");
  // CAN.setSPIFrequency(1e6);
  // CAN.setClockFrequency(8e6);
  // // start the CAN bus at 500 kbps
  // if (!CAN.begin(500E3)) {
  //   Serial.println("Starting CAN failed!");
  //   while (1);
  // }
}

void loop() {
  int curr_sens_luefter_reading= 0;
  float_t strom_sensor_luefter_strom = 0;

  curr_sens_luefter_reading = analogRead(A0);
  strom_sensor_luefter_strom = ((float_t)(curr_sens_luefter_reading * CURR_SENS_ABSOLUTE_MEASUREMENT_RANGE) / (float_t)CURR_SENS_ADC_BIT_RESOLUTION) + (float_t)CURR_SENS_MEASUREMENT_RANGE_OFFSET;

  // Ausgabe der Ergebnisse am Seriellen Monitor
  Serial.print("Luefterstrom = " ); 
  Serial.print(strom_sensor_luefter_strom, 3); 
  Serial.print(" [A]    "); 
  Serial.print(curr_sens_luefter_reading); 
  Serial.println();


  // temp_sensors.requestTemperatures();
  // Serial.print("Sensor 0: ");
  // printTemperature(temp_sens_0_addr);
  // Serial.println();
  // Serial.print("Sensor 1: ");
  // printTemperature(temp_sens_1_addr);
  // Serial.println();
  // Serial.print("Sensor 2: ");
  // printTemperature(temp_sens_2_addr);
  // Serial.println();
  // Serial.println();
  delay(100);
  
  // Send command to all the sensors for temperature conversion
  // temp_sensors.requestTemperatures(); 
  
  // // Display temperature from each sensor
  // for (int i = 0;  i < device_count;  i++)
  // {
  //   Serial.print("Sensor ");
  //   Serial.print(i+1);
  //   Serial.print(" : ");
  //   temp_in_grad = temp_sensors.getTempCByIndex(i);
  //   Serial.print(temp_in_grad);
  //   Serial.print("C");
  // }
  
  // Serial.println("");
  // delay(1000);
  
  // delay(2000);
  //shift_register_obj.set(1, HIGH);
  // shift_register_obj.set(2, HIGH);
  // shift_register_obj.set(3, HIGH);
  // shift_register_obj.set(4, HIGH);
  // shift_register_obj.set(5, HIGH);
  // shift_register_obj.set(6, HIGH);
  // shift_register_obj.set(7, HIGH);
  // shift_register_obj.set(8, HIGH);
  // shift_register_obj.set(8, HIGH);

  // shift_register_obj.set(10, HIGH);
  // shift_register_obj.set(11, HIGH);
  // shift_register_obj.set(12, HIGH);
  // shift_register_obj.set(13, HIGH);
  // shift_register_obj.set(14, HIGH);
  // shift_register_obj.set(15, HIGH);
  //sr->set(ouput_register_1, HIGH);
  //const uint8_t const_output_register_1 = ouput_register_1;
  //sr->setAll(&ouput_register_1);
  //ouput_register_1++;


  // delay(10000);
  // if(schalterzustand == 1)
  // {
  //   schalterzustand = 0;
  // }
  // else
  // {
  //   schalterzustand = 1;
  // }
  // digitalWrite(D1, schalterzustand);
  // // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  // Serial.print("Sending packet ... ");

  // CAN.beginPacket(0x12);
  // CAN.write('h');
  // CAN.write('e');
  // CAN.write('l');
  // CAN.write('l');
  // CAN.write('o');
  // CAN.endPacket();

  // Serial.println("done");

  // delay(1000);

  // // send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
  // Serial.print("Sending extended packet ... ");

  // CAN.beginExtendedPacket(0xabcdef);
  // CAN.write('w');
  // CAN.write('o');
  // CAN.write('r');
  // CAN.write('l');
  // CAN.write('d');
  // CAN.endPacket();

  // Serial.println("done");

  // delay(1000);
}


//############## Funktionen ################################
void printAddress(DeviceAddress deviceAddress)
{ 
  for (uint8_t i = 0; i < 8; i++)
  {
    Serial.print("0x");
    if (deviceAddress[i] < 0x10) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
    if (i < 7) Serial.print(", ");
  }
  Serial.println("");
}

void printTemperature(DeviceAddress deviceAddress)
{
  float tempC = temp_sensors.getTempC(deviceAddress);
  Serial.print(tempC);
  Serial.print("C");
}