#include <Arduino.h>
#include <CAN.h>
#include <ShiftRegister74HC595.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ########### Defines #######################

// ++++ Für die Schieberegister ++++
#define NUMBER_OF_SHIFT_REGISTERS (2) // Anzahl der 8bit Shift Register die hintereinander geschaltet sind
#define SHIFT_REGISTER_DATA_PIN_NO (4) // GPIO Pin Nummer über den die Daten für das Register eingestellt werden
#define SHIFT_REGISTER_CLOCK_PIN_NO (5) // GPIO Pin Nummer über den die Pulse zum Übernehmen der Daten aus SHIFT_REGISTER_DATA_PIN_NO übertragen werden 
#define SHIFT_REGISTER_OUTPUT_PIN_NO (0) // GPIO Pin Nummer für den Latch Pin. Wenn hier ein Puls zu sehen ist werden die Daten aus dem Register in den Output übernommen

// ++++ Für die Temperatursensoren ++++
#define ONE_WIRE_BUS (2) // Daten werden über diesen Pin gesendet und empfangen (PIN 2 entspricht D4)
#define TEMP_SENS_ADDR_0 {0x28, 0xDC, 0x91, 0x82, 0x31, 0x21, 0x03, 0x91} // One Wire Bus Adresse von Sensor 0
#define TEMP_SENS_ADDR_1 {0x28, 0xFB, 0x90, 0x9D, 0x31, 0x21, 0x03, 0xCD} // One Wire Bus Adresse von Sensor 1
#define TEMP_SENS_ADDR_2 {0x28, 0xA7, 0x82, 0x60, 0x31, 0x21, 0x03, 0x8D} // One Wire Bus Adresse von Sensor 2

// ++++ Für die Strommessungen ++++++++++
#define CURR_SENS_ABSOLUTE_MEASUREMENT_RANGE (10) // [A] Strommessspanne von -5 bis 5 Ampere macht 10A absolut
#define CURR_SENS_ADC_BIT_RESOLUTION (1024) // Wertebereich am ADC von 0 bis 1024
#define CURR_SENS_MEASUREMENT_RANGE_OFFSET (-5) // [A] Messwertbereich geht nicht von 0 bis 10 sondern von -5 bis 5, muss also um -5 verschoben werden

// ++++ Für die CAN Kommunikation ++++++++
#define SPI_CS_PIN_FOR_CAN_MPC2515 (15) // [] Pin Nummer des chip select pins der SPI Kommunikation zwischen D1 Mini und dem MPC2515 CAN Modul

#define CAN_LIB_RET_VAL_ERROR (0) // Return Wert der CAN Bibliothek der anzeigt der einen Fehler signalisiert
#define CAN_LIB_RET_VAL_OK (1) // Return Wert der CAN Bibliothek der anzeigt der signalisiert das alles in Ordnung ist

// IDs von Battery Pack 0 gehen von 0...9, von Pack 1 von 10...19 und so weiter
#define CAN_ID_PAKET_0  (0) // Paket ID (Arbitrierungs ID) für die CAN Kommunikation (anhand dieser Nummer wird das Paket eindeutig identifiziert)
#define CAN_ID_PAKET_1  (1) // Paket ID (Arbitrierungs ID) für die CAN Kommunikation (anhand dieser Nummer wird das Paket eindeutig identifiziert)
#define CAN_ID_PAKET_2  (2) // Paket ID (Arbitrierungs ID) für die CAN Kommunikation (anhand dieser Nummer wird das Paket eindeutig identifiziert)

// ++++ Error Codes ++++++++
#define ERROR_CODE_NO_ERROR                         (255u) // Kein Fehler ist aufgetreten
#define ERROR_CODE_INIT                             (  1u) // Init Wert für die Fehlerstatus
#define ERROR_CODE_CANNOT_CONNECT_TO_CAN            (  2u) // Can communication konnte nicht aufgesetzt werden (checke das CAN board (Kondensator kaput?) oder die Verkabelung vom SPI des D1 Mini zum CAN Board (MPC2515))
#define ERROR_CODE_AT_LEAST_ONE_BEGIN_OR_END_FAILED (  3u) // Bei mindestens einem der CAN Pakete ist es beim Aufrufen der Funktion "begin" oder "end" zu einem Fehler gekommen
#define ERROR_CODE_AT_LEAST_ONE_CAN_WRITE_FAILED    (  4u) // Bei mindestens einem der CAN Pakete ist es beim Aufrufen der Funktion "write" zu einem Fehler gekommen

// ++++ Fixed Point Umwandlung ++++++
/// Fixed-point Format: 7.9 (16-bit)
#define FIXED_POINT_FRACTIONAL_BITS_MAX128 (9) // 7 Bits für die Zahl vor dem Komma (maximal 128) und 9 Bits für die Zahl nach dem Komma (Auflösung von 0,002)
#define FIXED_POINT_FRACTIONAL_BITS_MAX64 (10) // 6 Bits für die Zahl vor dem Komma (maximal 64) und 10 Bits für die Zahl nach dem Komma (Auflösung von 0,001)
#define FIXED_POINT_FRACTIONAL_BITS_MAX32 (11) // 5 Bits für die Zahl vor dem Komma (maximal 32) und 11 Bits für die Zahl nach dem Komma (Auflösung von 0,0005)

//############ Globale Variablen ###############
// Variablen für das Relais Board
int schalterzustand = 1;

// ++++ Variablen für das Shift Register ++++
uint8_t ouput_register_1 = 0u;
ShiftRegister74HC595<NUMBER_OF_SHIFT_REGISTERS> shift_register_obj;

// ++++ Für die Temperatursensoren ++++
OneWire one_wire(ONE_WIRE_BUS); // Erzeuge ein oneWire Objekt um mit einem oneWire Gerät zu kommunizieren
DallasTemperature temp_sensors(&one_wire); //Erzeuge ein DallasTemperature Objekt mit Verweis auf das zu nutzende oneWire Objekt
int16_t i16_device_count = 0;
float f_temp_in_grad;
DeviceAddress Thermometer;
uint8_t u8_temp_sens_0_addr[8] = TEMP_SENS_ADDR_0;
uint8_t u8_temp_sens_1_addr[8] = TEMP_SENS_ADDR_1;
uint8_t u8_temp_sens_2_addr[8] = TEMP_SENS_ADDR_2;

// ++++ Variablen für den ADC Kanal Schalter (CD74HC4067) ++++++++
const uint8_t number_of_channel_select_pins=4;
uint8_t adc_channel_pin_mapping[16][number_of_channel_select_pins]={
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
  };

//############# Strukturen ############
typedef struct t_batt_pack_data {
	float_t f_temp_sens_0_val; // [Grad Celsius] Gemessene Temperatur von Temperatursensor 0
  float_t f_temp_sens_1_val; // [Grad Celsius] Gemessene Temperatur von Temperatursensor 1
  float_t f_temp_sens_2_val; // [Grad Celsius] Gemessene Temperatur von Temperatursensor 2
  float_t f_fan_current; // [A] Gemessene Stromstaerke der Luefterversorgungsleitung
  float_t f_heating_current; // [A] Gemessene Stromstärke der Luefterversorgungsleitung
  float_t f_voltage_sens_battery_pack; // [V] Gemessene Spannung an den äußeren Klemmen des Battery Packs
  uint16_t u16_status_bit_field; // Statusbitfeld der Batterie (Bit 0: Luefter geschaltet, Bit 1: Heizung geschaltet, Bit 2: Batterie voll, Bit 3: Batterie leer)
  uint8_t u8_error_code; // Error codes wie in den defines definiert  
}t_batt_pack_data;

//############# Funktionsprototypen ############
void printAddress(DeviceAddress deviceAddress);
void printTemperature(DeviceAddress deviceAddress);
void select_adc_channel(uint8_t channel_number);
void select_adc_channel_no_update(uint8_t channel_number);
void split_16bit_number_into_8bit(uint16_t u16_input_val, uint8_t au8_output_vals[2]);
uint8_t can_send_batt_pack_data(t_batt_pack_data *ps_batt_pack_data);

//############# Inline Funktionen ############
inline float_t fixed_to_float(uint16_t u16_input, uint8_t u8_fixed_point_fractional_bits)
{
    return ((float_t)u16_input / (float_t)(1 << u8_fixed_point_fractional_bits));
}

inline uint16_t float_to_fixed(float_t input, uint8_t u8_fixed_point_fractional_bits)
{
    return (uint16_t)(round(input * (1 << u8_fixed_point_fractional_bits)));
}

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
  //temp_sensors.begin();

  // ACHTUNG: RX und TX Pin werden als GPIOs umdefiniert!
  shift_register_obj.begin(SHIFT_REGISTER_DATA_PIN_NO, SHIFT_REGISTER_CLOCK_PIN_NO, SHIFT_REGISTER_OUTPUT_PIN_NO);

  // pinMode(D1, OUTPUT);
  // digitalWrite(D1, schalterzustand);
  // Serial.begin(9600);
  // while (!Serial);

  Serial.println("CAN Sender");
  CAN.setPins(SPI_CS_PIN_FOR_CAN_MPC2515, 2); // Interrupt PIN wird nicht genutzt und kann auf default 2 stehen bleiben
  CAN.setSPIFrequency(1e6);
  CAN.setClockFrequency(8e6);
  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  t_batt_pack_data s_batt_pack_data;
  int16_t curr_sens_luefter_reading_1= 0;
  int16_t curr_sens_luefter_reading_2= 0;
  uint8_t u8_status = ERROR_CODE_INIT;
  //float_t strom_sensor_luefter_strom = 0;

  // Setzt alle Daten in s_batt_pack_data auf 0
  memset(&s_batt_pack_data, 0, sizeof(s_batt_pack_data));

  select_adc_channel(2);
  curr_sens_luefter_reading_1 = analogRead(A0);
  delay(2000);
  //strom_sensor_luefter_strom = ((float_t)(curr_sens_luefter_reading * CURR_SENS_ABSOLUTE_MEASUREMENT_RANGE) / (float_t)CURR_SENS_ADC_BIT_RESOLUTION) + (float_t)CURR_SENS_MEASUREMENT_RANGE_OFFSET;
  select_adc_channel(4);
  curr_sens_luefter_reading_2 = analogRead(A0);

  // Ausgabe der Ergebnisse am Seriellen Monitor
  Serial.print(curr_sens_luefter_reading_1); 
  Serial.print("    ");
  Serial.print(curr_sens_luefter_reading_2); 
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
  //delay(2000);
  
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

  // Sende gesammelte Daten über CAN
  u8_status = can_send_batt_pack_data(&s_batt_pack_data);

}


//############## Funktionen ################################

//+++++ Funktionen für die Temperatursensoren +++++++++++++
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

//+++++ Funktionen für den ADC Channel Umschalter +++++++++++++
void select_adc_channel(uint8_t channel_number)
{
  for(uint8_t pin_number=0; pin_number < number_of_channel_select_pins; pin_number++)
  {
    // Setze den Schaltzustand im internen Register der Schiftregister
    shift_register_obj.setNoUpdate(pin_number, adc_channel_pin_mapping[channel_number][pin_number]);
  }
  // Übernehme den Schaltzustand der internen Register in die Ausgangspins
  shift_register_obj.updateRegisters();
}

void select_adc_channel_no_update(uint8_t channel_number)
{
  for(uint8_t pin_number=0; pin_number < number_of_channel_select_pins; pin_number++)
  {
    // Setze den Schaltzustand im internen Register der Schiftregister
    shift_register_obj.setNoUpdate(pin_number, adc_channel_pin_mapping[channel_number][pin_number]);
  }
}

void split_16bit_number_into_8bit(uint16_t u16_input_val, uint8_t au8_output_vals[2])
{
  au8_output_vals[0] = (u16_input_val >> 0) & 0xFF;  // shift um 0 Bit tut nichts, aber aus didaktischen Gruenden hier geschrieben
  au8_output_vals[1] = (u16_input_val >> 8) & 0xFF; 
}

uint8_t can_send_batt_pack_data(t_batt_pack_data *ps_batt_pack_data)
{
  uint8_t u8_status = ERROR_CODE_INIT;
  int i_can_begin_end_ret_val = 0;
  size_t can_write_ret_val = 0u;
  uint16_t u16_from_float;
  uint8_t au8_split_from_u16[2];

  // Paket 0 initialisieren, Werte berechnen, in das Paket schreiben und das Paket abschliessen
  i_can_begin_end_ret_val = CAN.beginPacket(CAN_ID_PAKET_0);
  if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_OK)
  {
    // Temperatursensor 0
    u16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_sens_0_val, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit(u16_from_float, au8_split_from_u16);
    can_write_ret_val |= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val |= CAN.write(au8_split_from_u16[1]);
    // Temperatursensor 1
    u16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_sens_1_val, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit(u16_from_float, au8_split_from_u16);
    can_write_ret_val |= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val |= CAN.write(au8_split_from_u16[1]);
    // Temperatursensor 2
    u16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_sens_2_val, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit(u16_from_float, au8_split_from_u16);
    can_write_ret_val |= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val |= CAN.write(au8_split_from_u16[1]);
    // Lüfterstrom
    u16_from_float = float_to_fixed(ps_batt_pack_data->f_fan_current, FIXED_POINT_FRACTIONAL_BITS_MAX32);
    split_16bit_number_into_8bit(u16_from_float, au8_split_from_u16);
    can_write_ret_val |= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val |= CAN.write(au8_split_from_u16[1]);
    // Paket abschliessen
    i_can_begin_end_ret_val |= CAN.endPacket();
    Serial.println("Paket 0 gesendet");
  }

  // TODO: Ändern auf Error Bitfeld um Fehler nicht zu maskieren
  if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_ERROR)
  {
    u8_status = ERROR_CODE_AT_LEAST_ONE_BEGIN_OR_END_FAILED;
  }
  if(can_write_ret_val == CAN_LIB_RET_VAL_ERROR)
  {
    u8_status = ERROR_CODE_AT_LEAST_ONE_CAN_WRITE_FAILED;
  }

  return u8_status;
}