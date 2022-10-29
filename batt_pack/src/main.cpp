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

// ++++ Für die ADC Messungen ++++++++++
#define CURR_SENS_ABSOLUTE_MEASUREMENT_RANGE (10.0f)  // [A] Strommessspanne von -5 bis 5 Ampere macht 10A absolut
#define CURR_SENS_ADC_BIT_RESOLUTION (1024)           // Wertebereich am ADC von 0 bis 1024
#define CURR_SENS_MEASUREMENT_RANGE_OFFSET (-5.0f)    // [A] Messwertbereich geht nicht von 0 bis 10 sondern von -5 bis 5, muss also um -5 verschoben werden
#define CURR_SENS_ADC_ZERO_POINT_CORRECTION (-13.0f)  // [] Korrektur des Nullpunkts, sodass bei 0A tatsaechlich auch 0A gemessen werden

#define VOLTAGE_SENS_BATTERY_PACK_RANGE (65.0f)       // [V] Messspanne fuer die Messung der Battery Pack Spannung
#define VOLTAGE_SENS_MEASUREMENT_RANGE_OFFSET (0.0f)  // [V] Messoffset (Nullpunktverschiebung) fuer die Messung der Battery Pack Spannung - kein Offset vorhanden
#define VOLTAGE_SENS_ADC_ZERO_POINT_CORRECTION (-13.0f)  // [] Korrektur des Nullpunkts, sodass bei 0V tatsaechlich auch 0V gemessen werden

#define NUMBER_OF_ADC_SAMPLES_PER_MEASUREMENT (5u) // [] Anzahl an Wiederholungen von ADC Messungen die anschliessend gemitte

// ++++ Für die CAN Kommunikation ++++++++
#define SPI_CS_PIN_FOR_CAN_MPC2515 (15) // [] Pin Nummer des chip select pins der SPI Kommunikation zwischen D1 Mini und dem MPC2515 CAN Modul

#define CAN_LIB_RET_VAL_ERROR (0) // Return Wert der CAN Bibliothek der anzeigt der einen Fehler signalisiert
#define CAN_LIB_RET_VAL_OK (1) // Return Wert der CAN Bibliothek der anzeigt der signalisiert das alles in Ordnung ist

// IDs von Battery Pack 0 gehen von 0...9, von Pack 1 von 10...19 und so weiter
#define CAN_ID_PAKET_0  (0) // Paket ID (Arbitrierungs ID) für die CAN Kommunikation (anhand dieser Nummer wird das Paket eindeutig identifiziert)
#define CAN_ID_PAKET_1  (1) // Paket ID (Arbitrierungs ID) für die CAN Kommunikation (anhand dieser Nummer wird das Paket eindeutig identifiziert)
#define CAN_ID_PAKET_2  (2) // Paket ID (Arbitrierungs ID) für die CAN Kommunikation (anhand dieser Nummer wird das Paket eindeutig identifiziert)

// ++++ Error Codes ++++++++
#define ERROR_BITFIELD_INIT                            (  0u) // Kein Fehler ist aufgetreten
#define ERROR_BIT_CANNOT_CONNECT_TO_CAN                (  0u) // Can communication konnte nicht aufgesetzt werden (checke das CAN board (Kondensator kaput?) oder die Verkabelung vom SPI des D1 Mini zum CAN Board (MPC2515))
#define ERROR_BIT_AT_LEAST_ONE_CAN_BEGIN_OR_END_FAILED (  1u) // Bei mindestens einem der CAN Pakete ist es beim Aufrufen der Funktion "begin" oder "end" zu einem Fehler gekommen
#define ERROR_BIT_AT_LEAST_ONE_CAN_WRITE_FAILED        (  2u) // Bei mindestens einem der CAN Pakete ist es beim Aufrufen der Funktion "write" zu einem Fehler gekommen
#define ERROR_BIT_TEMP_SENS_0_DISCONNECTED             (  3u) // Temperatursensor 0 nicht mehr erreichbar
#define ERROR_BIT_TEMP_SENS_1_DISCONNECTED             (  4u) // Temperatursensor 1 nicht mehr erreichbar
#define ERROR_BIT_TEMP_SENS_2_DISCONNECTED             (  5u) // Temperatursensor 2 nicht mehr erreichbar

// ++++ Fixed Point Umwandlung ++++++
/// Fixed-point Format: 7.9 (16-bit)
#define FIXED_POINT_FRACTIONAL_BITS_MAX128 (9) // 7 Bits für die Zahl vor dem Komma (maximal 128) und 9 Bits für die Zahl nach dem Komma (Auflösung von 0,002)
#define FIXED_POINT_FRACTIONAL_BITS_MAX64 (10) // 6 Bits für die Zahl vor dem Komma (maximal 64) und 10 Bits für die Zahl nach dem Komma (Auflösung von 0,001)
#define FIXED_POINT_FRACTIONAL_BITS_MAX32 (11) // 5 Bits für die Zahl vor dem Komma (maximal 32) und 11 Bits für die Zahl nach dem Komma (Auflösung von 0,0005)

// ++++ Hilfsdefines ++++
#define SMALL_FLOAT_VAL (0.0001f) // Kleiner float Wert der bei float Gleichheitsprüfungen als Toleranzwert dient
#define DEBUG_PRINT_ON (1) // Setze ungleich Null wenn über serielle Schnittstelle Debuginformationen ausgegeben werden sollen und 0 wenn nicht
//############ Globale Variablen ###############
uint32_t u32_micros_script_duration = 0ul;
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
uint8_t au8_temp_sens_0_addr[8] = TEMP_SENS_ADDR_0;
uint8_t au8_temp_sens_1_addr[8] = TEMP_SENS_ADDR_1;
uint8_t au8_temp_sens_2_addr[8] = TEMP_SENS_ADDR_2;

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
  uint16_t u16_status_bitfield; /* Statusbitfeld der Batterie (Bit 0: Luefter geschaltet, 
                                                               Bit 1: Heizung geschaltet, 
                                                               Bit 2: Batterie voll, 
                                                               Bit 3: Batterie leer, 
                                                               Bit 4: Daten über WIFI gesendet, 
                                                               Bit 5: Warnungsbit: Luefter oder Heizung aktiviert bei niedrigem Ladezustand,
                                                               Rest Reservebits) */
  uint16_t u16_error_bitfield; // Error codes wie in den defines definiert  
}t_batt_pack_data;

//############# Funktionsprototypen ############
void printAddress(DeviceAddress deviceAddress);
void printTemperature(DeviceAddress deviceAddress);
void select_adc_channel(uint8_t channel_number);
void select_adc_channel_no_update(uint8_t channel_number);
uint16_t can_send_batt_pack_data(t_batt_pack_data *ps_batt_pack_data);
uint16_t read_temperatures_from_all_connected_sensors(t_batt_pack_data *ps_batt_pack_data);
void read_all_ADC_measurement_values(t_batt_pack_data *ps_batt_pack_data);
float_t get_mean_of_n_ADC_samples(uint16_t u16_nof_samples);

#if(DEBUG_PRINT_ON)
void print_temperatures_from_all_connected_sensors(t_batt_pack_data *ps_batt_pack_data);
#endif
//############# Inline Funktionen ############
// inline float_t fixed_to_float(uint16_t u16_input, uint8_t u8_fixed_point_fractional_bits)
// {
//     return ((float_t)u16_input / (float_t)(1 << u8_fixed_point_fractional_bits));
// }

// inline uint16_t float_to_fixed_uint16(float_t input, uint8_t u8_fixed_point_fractional_bits)
// {
//     return (uint16_t)(round(input * (1 << u8_fixed_point_fractional_bits)));
// }

// inline int16_t float_to_fixed_int16(float_t input, uint8_t u8_fixed_point_fractional_bits)
// {
//     return (int16_t)(round(input * (1 << u8_fixed_point_fractional_bits)));
// }

inline int16_t float_to_fixed(float_t input, uint8_t u8_fixed_point_fractional_bits)
{
    float_t f_a = input * pow(2.0f, (int8_t)u8_fixed_point_fractional_bits);
    int16_t i_b = (int16_t)(round(f_a));
    if (f_a < 0.0f)
    {
        // Naechsten drei Zeilen wandeln b ins zweier Komplement
        i_b = abs(i_b);
        i_b = ~i_b;
        i_b = i_b + 1;
    }
    return i_b;
}

inline float_t fixed_to_float(int16_t i16_input, uint8_t u8_fixed_point_fractional_bits)
{
    int16_t i16_c = abs(i16_input);
    int16_t i_sign = 1;
    if (i16_input < 0)
    {
        // Naechsten drei Zeilen wandeln vom zweier Komplement zurueck
        i16_c = i16_input - 1;
        i16_c = ~i16_c;
        i_sign = -1;
    }
    float_t f_output = (1.0f * (float_t)i16_c) / pow(2.0f, (int16_t)u8_fixed_point_fractional_bits);
    f_output *= (float_t)i_sign;
    return f_output;
}

inline void set_bit_16bit(uint16_t u16_bitfield, uint8_t u8_bit_number)
{
  u16_bitfield |= 1u << u8_bit_number;
}

inline void clear_bit_16bit(uint16_t u16_bitfield, uint8_t u8_bit_number)
{
  u16_bitfield &= ~(1u << u8_bit_number);
}

// Gibt "True" zurueck wenn das Bit an der Stelle "u8_bit_number" gesetzt ist
inline bool check_bit_16bit(uint16_t u16_bitfield, uint8_t u8_bit_number)
{
  return (bool)((u16_bitfield >> u8_bit_number) & 1u);
}

inline void split_16bit_number_into_8bit_int16(int16_t i16_input_val, uint8_t au8_output_vals[2])
{
  au8_output_vals[0] = (i16_input_val >> 0) & 0xFF;  // shift um 0 Bit tut nichts, aber aus didaktischen Gruenden hier geschrieben
  au8_output_vals[1] = (i16_input_val >> 8) & 0xFF; 
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
  temp_sensors.begin();

  // ACHTUNG: RX und TX Pin werden als GPIOs umdefiniert!
  shift_register_obj.begin(SHIFT_REGISTER_DATA_PIN_NO, SHIFT_REGISTER_CLOCK_PIN_NO, SHIFT_REGISTER_OUTPUT_PIN_NO);
  shift_register_obj.set(7, LOW);
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
  uint16_t u16_status = ERROR_BITFIELD_INIT;
  uint32_t u32_microseconds_start = micros();

  // Setzt alle Daten in s_batt_pack_data auf 0
  memset(&s_batt_pack_data, 0, sizeof(s_batt_pack_data));

  // ############# Auslesen der analogen Messwerte #############################
  read_all_ADC_measurement_values(&s_batt_pack_data);
  // ############# Einlesen der Temperatursensoren #############################
  u16_status |= read_temperatures_from_all_connected_sensors(&s_batt_pack_data);
  
  #if(DEBUG_PRINT_ON)
  print_temperatures_from_all_connected_sensors(&s_batt_pack_data);
  #endif

  // ############# Fehlerbehandlung lokal fuer dieses Battery Pack #############################
  /* Warnung raushauen wenn die Batterie fast leer ist und der Luefter oder die Heizung aktiviert wurden.
  Dafuer das Statusbit nutzen TODO*/

  /* Hier auch plausibilitaetschecks durchfuehren wie z.B. Luefter aus aber trotzdem fliesst Strom - same with Heizung - */

  // ############# Sende gesammelte Daten über CAN #############################
  u16_status = can_send_batt_pack_data(&s_batt_pack_data);
  // Uebertrage moegliche Fehlerbits aus der Funktion in das "globale" Fehlerbitfeld
  s_batt_pack_data.u16_error_bitfield |= u16_status;


  // ############# Fehlerbehandlung wenn CAN Kommunikation fehlschlaegt #############################
  /* Statusabfrage CAN SEND: Wenn keine Übertragung möglich TODO: Konzept überlegen
     1. Sofort die Daten versuchen über WIFI rauszusenden, damit man an die Fehlerbits kommt.
        Auf das ACK warten (mit einem Timeout) ob die Daten empfangen wurden.
        Wenn kein ACK kommt nach 1h erneut versuchen zu verbinden. Insgesamt 5 Versuche bis endgültig abgebrochen wird
        ueber WIFI - verbraucht sonst zuviel Strom. Ueber CAN wird einfach immer weiter versucht.
  */
  if(check_bit_16bit(s_batt_pack_data.u16_error_bitfield, ERROR_BIT_AT_LEAST_ONE_CAN_BEGIN_OR_END_FAILED) 
  || check_bit_16bit(s_batt_pack_data.u16_error_bitfield, ERROR_BIT_AT_LEAST_ONE_CAN_WRITE_FAILED)
  || check_bit_16bit(s_batt_pack_data.u16_error_bitfield, ERROR_BIT_CANNOT_CONNECT_TO_CAN))
  {
    // Hier die Behandlung für den fehlgeschlagenen CAN einfuegen
  }

  uint32_t u32_microseconds_end = micros();
  u32_micros_script_duration = u32_microseconds_end - u32_microseconds_start;

  #if(DEBUG_PRINT_ON)
  Serial.println("");
  Serial.print("Fehlerbitfeld Battery Pack: ");
  Serial.print(s_batt_pack_data.u16_error_bitfield);
  Serial.println("");
  Serial.print("Statusbitfeld Battery Pack: ");
  Serial.print(s_batt_pack_data.u16_status_bitfield);
  Serial.println("");
  Serial.print("Loop Laufzeit: ");
  Serial.print(u32_micros_script_duration);
  Serial.print("µs");
  Serial.println("");

  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  #endif

  delay(5000);
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

uint16_t read_temperatures_from_all_connected_sensors(t_batt_pack_data *ps_batt_pack_data)
{
  uint16_t u16_status = ERROR_BITFIELD_INIT;
  // Starten der Temperaturmessung an allen angeschlossenen Sensoren
  temp_sensors.requestTemperatures();
  // Schreiben der einzelnen Temperaturmesswerte in interne Datenstruktur
  ps_batt_pack_data->f_temp_sens_0_val = temp_sensors.getTempC(au8_temp_sens_0_addr);
  ps_batt_pack_data->f_temp_sens_1_val = temp_sensors.getTempC(au8_temp_sens_1_addr);
  ps_batt_pack_data->f_temp_sens_2_val = temp_sensors.getTempC(au8_temp_sens_2_addr);
  // Ueberpruefen auf Fehler und setzen der entsprechenden Fehlerbits
  if(fabs(ps_batt_pack_data->f_temp_sens_0_val - (float_t)DEVICE_DISCONNECTED_C) < SMALL_FLOAT_VAL)
  {
      set_bit_16bit(u16_status, ERROR_BIT_TEMP_SENS_0_DISCONNECTED);
  }
  else if (fabs(ps_batt_pack_data->f_temp_sens_1_val - (float_t)DEVICE_DISCONNECTED_C) < SMALL_FLOAT_VAL)
  {
    set_bit_16bit(u16_status, ERROR_BIT_TEMP_SENS_1_DISCONNECTED);
  }
  else if (fabs(ps_batt_pack_data->f_temp_sens_2_val - (float_t)DEVICE_DISCONNECTED_C) < SMALL_FLOAT_VAL)
  {
    set_bit_16bit(u16_status, ERROR_BIT_TEMP_SENS_2_DISCONNECTED);
  }
  else
  {
    /* No error occured - do nothing*/
  }
  return u16_status;
}

#if(DEBUG_PRINT_ON)
void print_temperatures_from_all_connected_sensors(t_batt_pack_data *ps_batt_pack_data)
{
  Serial.print("Temp Sensor 0: ");
  Serial.print(ps_batt_pack_data->f_temp_sens_0_val);
  Serial.print("C");
  Serial.println("");
  Serial.print("Temp Sensor 1: ");
  Serial.print(ps_batt_pack_data->f_temp_sens_1_val);
  Serial.print("C");
  Serial.println("");
  Serial.print("Temp Sensor 2: ");
  Serial.print(ps_batt_pack_data->f_temp_sens_2_val);
  Serial.print("C");
  Serial.println("");
}
#endif
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

//+++++ Funktionen fuer das Auslesen der ADC Messwerte +++++++++++++
void read_all_ADC_measurement_values(t_batt_pack_data *ps_batt_pack_data)
{
  float_t f_mean_adc_readings=0.0f;
  // Auslesen und berechnen des Luefterstroms
  select_adc_channel(0);
  delay(1); // Gibt dem ADC Zeit auf einen neuen Wert einzuschwingen
  f_mean_adc_readings = get_mean_of_n_ADC_samples(NUMBER_OF_ADC_SAMPLES_PER_MEASUREMENT);
  f_mean_adc_readings += CURR_SENS_ADC_ZERO_POINT_CORRECTION;
  ps_batt_pack_data->f_fan_current = ((f_mean_adc_readings * CURR_SENS_ABSOLUTE_MEASUREMENT_RANGE)/(float_t)CURR_SENS_ADC_BIT_RESOLUTION) + CURR_SENS_MEASUREMENT_RANGE_OFFSET;
  #if(DEBUG_PRINT_ON)
    Serial.println("");
    Serial.print("Luefterstrom (ADC Channel 0): ");
    Serial.print(f_mean_adc_readings, 3);
    Serial.print("   ");
    Serial.print(ps_batt_pack_data->f_fan_current, 3);
    Serial.print("A");
  #endif

  // Auslesen und berechnen des Heizungsstroms
  //strom_sensor_luefter_strom = ((float_t)(curr_sens_luefter_reading * CURR_SENS_ABSOLUTE_MEASUREMENT_RANGE) / (float_t)CURR_SENS_ADC_BIT_RESOLUTION) + (float_t)CURR_SENS_MEASUREMENT_RANGE_OFFSET;
  select_adc_channel(1);
  delay(1); // Gibt dem ADC Zeit auf einen neuen Wert einzuschwingen
  f_mean_adc_readings = get_mean_of_n_ADC_samples(NUMBER_OF_ADC_SAMPLES_PER_MEASUREMENT);
  f_mean_adc_readings += CURR_SENS_ADC_ZERO_POINT_CORRECTION;
  ps_batt_pack_data->f_heating_current = ((f_mean_adc_readings * CURR_SENS_ABSOLUTE_MEASUREMENT_RANGE)/(float_t)CURR_SENS_ADC_BIT_RESOLUTION) + CURR_SENS_MEASUREMENT_RANGE_OFFSET;
  #if(DEBUG_PRINT_ON)
    Serial.println("");
    Serial.print("Heizungsstrom (ADC Channel 1): ");
    Serial.print(f_mean_adc_readings, 3);
    Serial.print("   ");
    Serial.print(ps_batt_pack_data->f_heating_current, 3);
    Serial.print("A");
  #endif

  // Auslesen der Batteriepack Spannung
  select_adc_channel(2);
  delay(1); // Gibt dem ADC Zeit auf einen neuen Wert einzuschwingen
  f_mean_adc_readings = get_mean_of_n_ADC_samples(NUMBER_OF_ADC_SAMPLES_PER_MEASUREMENT);
  f_mean_adc_readings += VOLTAGE_SENS_ADC_ZERO_POINT_CORRECTION;
  ps_batt_pack_data->f_voltage_sens_battery_pack = ((f_mean_adc_readings * VOLTAGE_SENS_BATTERY_PACK_RANGE)/(float_t)CURR_SENS_ADC_BIT_RESOLUTION) + VOLTAGE_SENS_MEASUREMENT_RANGE_OFFSET;
  #if(DEBUG_PRINT_ON)
    Serial.println("");
    Serial.print("Batteriespannung (ADC Channel 3): ");
    Serial.print(f_mean_adc_readings, 3);
    Serial.print("   ");
    Serial.print(ps_batt_pack_data->f_voltage_sens_battery_pack, 2);
    Serial.print("V");
    Serial.println("");
  #endif
}

float_t get_mean_of_n_ADC_samples(uint16_t u16_nof_samples)
{
  int i_sum_of_readings = 0;
  for(uint16_t u16_sample_number=0; u16_sample_number < u16_nof_samples; u16_sample_number++)
  {
    i_sum_of_readings += analogRead(A0);
  }

  return (float_t)(i_sum_of_readings / u16_nof_samples);
}

//+++++ Funktionen für die CAN Kommunikation +++++++++++++

uint16_t can_send_batt_pack_data(t_batt_pack_data *ps_batt_pack_data)
{
  uint16_t u16_status = ERROR_BITFIELD_INIT;
  int i_can_begin_end_ret_val = CAN_LIB_RET_VAL_OK;
  size_t can_write_ret_val = 0u;
  int16_t i16_from_float;
  uint8_t au8_split_from_u16[2];

  // Paket 0 initialisieren, Werte berechnen, in das Paket schreiben und das Paket abschliessen
  i_can_begin_end_ret_val &= CAN.beginPacket(CAN_ID_PAKET_0);
  if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_OK)
  {
    // Temperatursensor 0
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_sens_0_val, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Temperatursensor 1
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_sens_1_val, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Temperatursensor 2
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_sens_2_val, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Luefterstrom
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_fan_current, FIXED_POINT_FRACTIONAL_BITS_MAX32);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Paket abschliessen
    i_can_begin_end_ret_val &= CAN.endPacket();
    #if(DEBUG_PRINT_ON)
    Serial.println("");
    Serial.println("Paket 0 gesendet");
    #endif
  }

  // Paket 1 initialisieren, Werte berechnen, in das Paket schreiben und das Paket abschliessen
  i_can_begin_end_ret_val &= CAN.beginPacket(CAN_ID_PAKET_1);
  if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_OK)
  {
    // Heizungsstrom
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_heating_current, FIXED_POINT_FRACTIONAL_BITS_MAX32);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Spannung des Battery Packs
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_voltage_sens_battery_pack, FIXED_POINT_FRACTIONAL_BITS_MAX64);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Status Bitfeld
    split_16bit_number_into_8bit_int16(int16_t(ps_batt_pack_data->u16_status_bitfield), au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif

    // Fuelle die letzten moeglichen Fehler in das Errorbitfeld - dieser Pfad wird gegebenenfalls nicht erreicht, weswegen der Fehlersetzcode scheinbar doppelt ist
    // ACHTUNG: Das Setzen der Fehlerbits muss immer im letzten Paket erfolgen. Wenn zukuenftig mehr als zwei Pakete versendet werden muss dieser Code ins letzte Paket kopiert werden!!!
    // Setzen der Fehlerbits der erkannten Fehler
    if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_ERROR)
    {
      set_bit_16bit(ps_batt_pack_data->u16_error_bitfield, ERROR_BIT_AT_LEAST_ONE_CAN_BEGIN_OR_END_FAILED);
    }
    if(can_write_ret_val == CAN_LIB_RET_VAL_ERROR)
    {
      set_bit_16bit(ps_batt_pack_data->u16_error_bitfield, ERROR_BIT_AT_LEAST_ONE_CAN_WRITE_FAILED);
    }
    // Fehler Bitfeld
    split_16bit_number_into_8bit_int16(int16_t(ps_batt_pack_data->u16_error_bitfield), au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Paket abschliessen - Achtung: moeglicher Fehler in CAN.endPacket wird nicht erkannt und übermittelt
    CAN.endPacket();
    #if(DEBUG_PRINT_ON)
    Serial.println("");
    Serial.println("Paket 1 gesendet");
    #endif
  }

  // Setzen der Fehlerbits der erkannten Fehler
  if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_ERROR)
  {
    set_bit_16bit(u16_status, ERROR_BIT_AT_LEAST_ONE_CAN_BEGIN_OR_END_FAILED);
  }
  if(can_write_ret_val == CAN_LIB_RET_VAL_ERROR)
  {
    set_bit_16bit(u16_status, ERROR_BIT_AT_LEAST_ONE_CAN_WRITE_FAILED);
  }

  #if(DEBUG_PRINT_ON)
  Serial.print("");
  Serial.print("Fehlerbitfeld lokale CAN Send Funktion: ");
  Serial.print(u16_status);
  Serial.print("");
  #endif

  return u16_status;
}

