#include <Arduino.h>
#include <CAN.h>
#include <ShiftRegister74HC595.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <espMqttClient.h>
#include <string>
#include <TimeLib.h>
#include <math.h>
#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>

// ########### Defines #######################
// ++++ Für die Schieberegister ++++
#define NUMBER_OF_SHIFT_REGISTERS (2) // Anzahl der 8bit Shift Register die hintereinander geschaltet sind
#define SHIFT_REGISTER_DATA_PIN_NO (4) // GPIO Pin Nummer über den die Daten für das Register eingestellt werden
#define SHIFT_REGISTER_CLOCK_PIN_NO (5) // GPIO Pin Nummer über den die Pulse zum Übernehmen der Daten aus SHIFT_REGISTER_DATA_PIN_NO übertragen werden 
#define SHIFT_REGISTER_OUTPUT_PIN_NO (0) // GPIO Pin Nummer für den Latch Pin. Wenn hier ein Puls zu sehen ist werden die Daten aus dem Register in den Output übernommen

// ++++ Für die Temperatursensoren ++++
#define ONE_WIRE_BUS (2) // Daten werden über diesen Pin gesendet und empfangen (PIN 2 entspricht D4)
#define TEMP_SENS_RESOLUTION (12) // [9-12] 9bit bis 12bit Resolution: Je höher die Resolution desto genauer die Temperaturmessung aber je länger muss man auf das Ergebnis warten (750ms bei 12 bit)
#define TEMP_SENS_WAIT_TIME_NEXT_TEMP_READING_MILLIS (15000u) // [ms] Anzahl an Millisekunden Wartezeit zwischen zwei Temperaturmessungen
#define MAX_NOF_TEMP_SENSORS (6) // [] Maximale Anzahl von Temperatursensoren die angeschlossen werden können
// #define TEMP_SENS_ADDR_0 {0x28, 0xDC, 0x91, 0x82, 0x31, 0x21, 0x03, 0x91} // One Wire Bus Adresse von Sensor 0
// #define TEMP_SENS_ADDR_1 {0x28, 0xFB, 0x90, 0x9D, 0x31, 0x21, 0x03, 0xCD} // One Wire Bus Adresse von Sensor 1
// #define TEMP_SENS_ADDR_2 {0x28, 0xA7, 0x82, 0x60, 0x31, 0x21, 0x03, 0x8D} // One Wire Bus Adresse von Sensor 2

// ++++ Für die ADC Messungen ++++++++++
#define CURR_SENS_ABSOLUTE_MEASUREMENT_RANGE (28.0f)  // [A] Strommessspanne von -5 bis 5 Ampere macht 10A absolut
#define CURR_SENS_ADC_BIT_RESOLUTION (1024)           // Wertebereich am ADC von 0 bis 1024
#define CURR_SENS_MEASUREMENT_RANGE_OFFSET (-13.9f)    // [A] Messwertbereich geht nicht von 0 bis 10 sondern von -5 bis 5, muss also um -5 verschoben werden
#define CURR_SENS_ADC_ZERO_POINT_CORRECTION (3.0f)  // [] Korrektur des Nullpunkts, sodass bei 0A tatsaechlich auch 0A gemessen werden

#define VOLTAGE_SENS_BATTERY_PACK_RANGE (65.0f)       // [V] Messspanne fuer die Messung der Battery Pack Spannung
#define VOLTAGE_SENS_MEASUREMENT_RANGE_OFFSET (0.0f)  // [V] Messoffset (Nullpunktverschiebung) fuer die Messung der Battery Pack Spannung - kein Offset vorhanden
#define VOLTAGE_SENS_ADC_ZERO_POINT_CORRECTION (-13.0f)  // [] Korrektur des Nullpunkts, sodass bei 0V tatsaechlich auch 0V gemessen werden

#define NUMBER_OF_ADC_SAMPLES_PER_MEASUREMENT (5u) // [] Anzahl an Wiederholungen von ADC Messungen die anschliessend gemitte

// ++++ Für die CAN Kommunikation ++++++++
#define SPI_CS_PIN_FOR_CAN_MPC2515 (15) // [] Pin Nummer des chip select pins der SPI Kommunikation zwischen D1 Mini und dem MPC2515 CAN Modul
#define RECEIVE_POLLING_NOF_SKIPPED_WAIT_MILLISECONDS (1u) // [>0] Am Ende eines Loopdurchlaufs wird gewartet bevor die nächste Loop startet. In der Zeit wird der CAN gepollt. Hier kann man einstellen wie häufig gepollt werden soll.
#define DEVICE_ID_IN_CAN_NETWORK (2u) // [0-255] Jedes Gerät am CAN Bus hat eine eigene einmalige ID. Battery Pack 0 hat die ID 2.
#define CAN_RECEIVE_FILTER_MASK  (0x7f8) // [] Filtermaske - laesst nur Botschaften für DEVICE_ID_IN_CAN_NETWORK durch in unserem Smarthome CAN Bus.
#define INPUT_BUFFER_SIZE_IN_BYTES  (64u) // [] Anzahl der Bytes des Inputbuffers. Eine CAN Botschaft enthält bis zu 8 Bytes Daten, sodass bei 64 bytes 8 volle Botschaften in den Zwischenspeicher passen.

#define CAN_LIB_RET_VAL_ERROR ((size_t)0) // Return Wert der CAN Bibliothek der anzeigt der einen Fehler signalisiert
#define CAN_LIB_RET_VAL_OK (1) // Return Wert der CAN Bibliothek der anzeigt der signalisiert das alles in Ordnung ist

#define CAN_REC_ARBITRATION_ID_1BYTE_SUB_ID  (16u) // Arbitrierungs ID für ein sub ID Byte für Pack 0 nach CAN Smarthome Doku
#define CAN_REC_ARBITRATION_ID_2BYTE_SUB_ID  (17u) // Arbitrierungs ID für 2 Sub ID Bytes für Pack 0 nach CAN Smarthome Doku
#define CAN_REC_2BYTE_SUB_ID_MSG_ID_GLOBALTIME  (1u) // Sub Botschafts ID für die Übertragung des globalen Zeitstempels

#define CAN_SEND_RASPI_ARBITRATION_ID_1BYTE_SUB_ID  (8u) // Arbitrierungs ID für ein sub ID Byte für Pack 0 zum Raspismarthome nach CAN Smarthome Doku
#define CAN_SEND_RASPI_ARBITRATION_ID_2BYTE_SUB_ID  (9u) // Arbitrierungs ID für 2 Sub ID Bytes für Pack 0 zum Raspismarthome nach CAN Smarthome Doku
#define CAN_SEND_RASPI_2BYTE_SUB_ID_MSG_ID_TEMP1  (1u) // Sub Botschafts ID für die Übertragung der ersten drei Temperaturen
#define CAN_SEND_RASPI_2BYTE_SUB_ID_MSG_ID_TEMP2  (2u) // Sub Botschafts ID für die Übertragung der zweiten drei Temperaturen
#define CAN_SEND_RASPI_2BYTE_SUB_ID_MSG_ID_CURRENT  (3u) // Sub Botschafts ID für die Übertragung der zwei gemessenen Ströme und der Batteriespannung
#define CAN_SEND_RASPI_2BYTE_SUB_ID_MSG_ID_STATUSERROR  (4u) // Sub Botschafts ID für die Übertragung des Status und der Error Bits

// WIFI
#define MAX_NOF_RECONNECT_TRIES_BEVORE_ECU_RESET (100u) // [0-254]
 
// MQTT
#define MILLISECONDS_TO_PUBLISH_KEEP_ALIVE (10000u)
#define MILLISECONDS_TO_PUBLISH_SIGNAL_STRENGTH (60000u)
#define MQTT_HOST IPAddress(192, 168, 178, 44)
#define MQTT_PORT 1883
#define MQTT_RETAIN_MESSAGE (true)
#define MQTT_DO_NOT_RETAIN_MESSAGE (false)
#define MQTT_QOS_0 (0)
#define MQTT_QOS_1 (1)
#define MQTT_QOS_2 (2)

// ++++ Error Codes ++++++++
#define ERROR_BITFIELD_INIT                            (  0u) // Kein Fehler ist aufgetreten
#define ERROR_BIT_CANNOT_CONNECT_TO_CAN                (  0u) // Can communication konnte nicht aufgesetzt werden (checke das CAN board (Kondensator kaput?) oder die Verkabelung vom SPI des D1 Mini zum CAN Board (MPC2515))
#define ERROR_BIT_AT_LEAST_ONE_CAN_BEGIN_OR_END_FAILED (  1u) // Bei mindestens einem der CAN Pakete ist es beim Aufrufen der Funktion "begin" oder "end" zu einem Fehler gekommen
#define ERROR_BIT_AT_LEAST_ONE_CAN_WRITE_FAILED        (  2u) // Bei mindestens einem der CAN Pakete ist es beim Aufrufen der Funktion "write" zu einem Fehler gekommen
#define ERROR_BIT_TEMP_SENS_DISCONNECTED               (  3u) // Einer der Temperatursensoren ist nicht mehr erreichbar
#define ERROR_BIT_POSSIBLE_DATA_LOSS_CAN               (  4u) // Auf dem CAN wurden mehr Bytes empfangen als vom Empfangsbuffer aufgenommen werden konnten
#define ERROR_BIT_UNKOWN_2BYTE_SUB_ID                  (  5u) // CAN hat eine Botschaft mit einer unbekannten 2 Byte Sub ID empfangen

// ++++ Fixed Point Umwandlung ++++++
/// Fixed-point Format: 7.9 (16-bit)
#define FIXED_POINT_FRACTIONAL_BITS_MAX128 (9) // 7 Bits für die Zahl vor dem Komma (maximal 128) und 9 Bits für die Zahl nach dem Komma (Auflösung von 0,002)
#define FIXED_POINT_FRACTIONAL_BITS_MAX64 (10) // 6 Bits für die Zahl vor dem Komma (maximal 64) und 10 Bits für die Zahl nach dem Komma (Auflösung von 0,001)
#define FIXED_POINT_FRACTIONAL_BITS_MAX32 (11) // 5 Bits für die Zahl vor dem Komma (maximal 32) und 11 Bits für die Zahl nach dem Komma (Auflösung von 0,0005)

// ++++ Steuerparamter Script ++++
#define NOF_MILLISECONDS_WAIT_TIME_FOR_NEXT_CYCLE (5000) // [ms] Wartezeit am Ende der Mainloop bevor der nächste Loopdurchlauf gestartet wird in Millisekunden


// ++++ Hilfsdefines ++++
#define SMALL_FLOAT_VAL (0.0001f) // Kleiner float Wert der bei float Gleichheitsprüfungen als Toleranzwert dient
#define DEBUG_PRINT_ON (1) // Setze ungleich Null wenn über serielle Schnittstelle Debuginformationen ausgegeben werden sollen und 0 wenn nicht
#define MILLISECONDS_FOR_ECU_REBOOT (75600000u)// [unsigned in 32 bit] Anzahl an Millisekunden nach denen sich die ECU neustartet
#define NO_SCHEDULED_ECU_REBOOT (false)// [bool] Wenn die ECU nicht zu einer bestimmten Uhrzeit (oder Datum etc.) neugestartet werden soll
#define NO_TIMELESS_SCHEDULED_ECU_REBOOT (true)// [bool] Wenn die ECU nicht nach Ablauf eines timers neugestartet werden soll
#define MILLISECONDS_FOR_AUTONOMOUS_MODE_WHEN_MISSING_MASTER (60000u)// [unsigned in 32 bit] Anzahl an Millisekunden nach denen in die ECU in den Automatikmodus wechselt wenn die globaltime Botschaft ausbleibt

//############ Globale Variablen ###############
// KONFIGURIERE HIER
// Wifi
const char* SSID = "Dieter seine weisse Box";
const char* PSK = "NS3e0853H1r117u";
// MQTT
const char* MQTT_BROKER = "raspismarthome";
const char* MQTT_DEVICE_NAME = "huehnerstall_esp8266_1";

uint32_t u32_micros_script_duration = 0ul;
// Variablen für das Relais Board
int schalterzustand = 1;

// +++++ Variablen für CAN ++++++++++++++++++++++
uint8_t au8_rx_buffer[INPUT_BUFFER_SIZE_IN_BYTES];
uint16_t u16_rx_can_ids;

// ++++ Variablen Programmflusskontrolle ++++++++
uint16_t u16_error_bitfield_after_can_send = 0u;

// ++++ Variablen für das Shift Register ++++
uint8_t ouput_register_1 = 0u;
ShiftRegister74HC595<NUMBER_OF_SHIFT_REGISTERS> shift_register_obj;

// ++++ Für die Temperatursensoren ++++
OneWire one_wire(ONE_WIRE_BUS); // Erzeuge ein oneWire Objekt um mit einem oneWire Gerät zu kommunizieren
DallasTemperature temp_sensors(&one_wire); //Erzeuge ein DallasTemperature Objekt mit Verweis auf das zu nutzende oneWire Objekt
DeviceAddress Thermometer;
uint32_t u32_temp_conversiontime = 0u;
uint32_t u32_last_temp_read_time[MAX_NOF_TEMP_SENSORS] = {0u};
float_t af_temp_in_grad[MAX_NOF_TEMP_SENSORS] = {-127.0f};
float_t f_temp_in_grad_last_publish = 0.0f;
uint8_t au8_temp_sens_addr[MAX_NOF_TEMP_SENSORS][8] = {0};
uint8_t u8_temp_sens_device_count = 0;

uint32_t u32_milliseconds_mqtt_keep_alive_starttime = 0u;
uint32_t u32_milliseconds_mqtt_signal_strength_starttime = 0u;

wl_status_t s_wifi_begin_status = WL_DISCONNECTED;
uint8_t u8_wifi_disconnect_cntr = 0u;
uint8_t u8_mqtt_keep_alive_cntr = 0u;
uint32_t u32_last_reboot_time = 0u;
bool b_autonomous_mode = false;
bool b_master_sent_reset_command = false;
uint32_t u32_last_master_alive_seen_time = 0u;

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
	float_t f_temp_busbar_plus; // [Grad Celsius] Gemessene Temperatur der Anschlussbusbar plus
  float_t f_temp_busbar_minus; // [Grad Celsius] Gemessene Temperatur der Anschlussbusbar minus
  float_t f_temp_cable_shoe_1;// [Grad Celsius] Gemessene Temperatur am Kabelschuh des internen Verbindungskabels (Verbindung zwischen oberen und unteren 8 Zellen)
  float_t f_temp_cable_shoe_2;// [Grad Celsius] Gemessene Temperatur am Kabelschuh des internen Verbindungskabels (Verbindung zwischen oberen und unteren 8 Zellen)
  float_t f_temp_air_heating_side;// [Grad Celsius] Gemessene Temperatur der Luft auf Seite der Heizung
  float_t f_temp_air_cold_side;// [Grad Celsius] Gemessene Temperatur der Luft auf der Seite ohne Heizung
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
uint16_t create_can_id_from_device_id(uint8_t u8_device_id);
void read_multi_temp_non_blocking_with_wait_time(uint32_t u32_wait_time_millis);
void poll_can_bus(t_batt_pack_data *ps_batt_pack_data);
void poll_can_bus_single(t_batt_pack_data *ps_batt_pack_data);
void autonomous_mode_handler();

#if(DEBUG_PRINT_ON)
void print_temperatures_from_all_connected_sensors(t_batt_pack_data *ps_batt_pack_data);
#endif

//############# Inline Funktionen ############
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

inline void set_bit_16bit(uint16_t *pu16_bitfield, uint8_t u8_bit_number)
{
  (*pu16_bitfield) |= 1u << u8_bit_number;
}

inline void clear_bit_16bit(uint16_t *pu16_bitfield, uint8_t u8_bit_number)
{
  (*pu16_bitfield) &= ~(1u << u8_bit_number);
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

inline uint16_t merge_8bit_numbers_into_16bit_uint16(uint8_t au8_input_vals[2])
{
  return (uint16_t)((au8_input_vals[0] << 8) | (au8_input_vals[1]));
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
  #if(DEBUG_PRINT_ON)
  // start serial port
  Serial.begin(9600);
  #endif

  // Start up the library
  temp_sensors.begin();
  u32_temp_conversiontime = (uint32_t)(750 / (1 << (12 - TEMP_SENS_RESOLUTION)));
  u8_temp_sens_device_count = temp_sensors.getDeviceCount();
  for (uint8_t i = 0;  i < u8_temp_sens_device_count;  i++)
  {
    #if(DEBUG_PRINT_ON)
    Serial.print("Sensor ");
    Serial.print((int)i+1);
    Serial.print(" : ");
    #endif
    temp_sensors.getAddress(au8_temp_sens_addr[i], i);
    u32_last_temp_read_time[i] = millis();
  }

  // ACHTUNG: RX und TX Pin werden als GPIOs umdefiniert!
  shift_register_obj.begin(SHIFT_REGISTER_DATA_PIN_NO, SHIFT_REGISTER_CLOCK_PIN_NO, SHIFT_REGISTER_OUTPUT_PIN_NO);
  shift_register_obj.set(7, LOW);
  // pinMode(D1, OUTPUT);
  // digitalWrite(D1, schalterzustand);
  // Serial.begin(9600);
  // while (!Serial);

  Serial.println("CAN Sender");
  CAN.setPins(SPI_CS_PIN_FOR_CAN_MPC2515, 2); // Interrupt PIN wird nicht genutzt und kann auf default 2 stehen bleiben
  CAN.setSPIFrequency(1000E3);
  CAN.setClockFrequency(8e6);
  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    /* TODO: Statt while Schleife eine Fehlermeldung über WIFI raussenden*/
    while (1);
  }
  // CAN Filterung der Botschaften - nur Botschaften mit dieser CAN id werden empfangen - alle anderen Botschaften werden verworfen um den Bearbeitungsaufwand zu verringern
  CAN.filter(create_can_id_from_device_id(DEVICE_ID_IN_CAN_NETWORK), CAN_RECEIVE_FILTER_MASK);
}

void loop() {
  t_batt_pack_data s_batt_pack_data;
  uint32_t u32_microseconds_start = micros();
  uint16_t u16_errors = ERROR_BITFIELD_INIT;

  // Setzt alle Daten in s_batt_pack_data auf 0
  memset(&s_batt_pack_data, 0, sizeof(s_batt_pack_data));
  // Übernehme die Fehler nach dem raussenden der CAN Pakete des letzten Zyklus für diesen Zyklus
  s_batt_pack_data.u16_error_bitfield = u16_error_bitfield_after_can_send;

  // ############# Auslesen der analogen Messwerte #############################
  read_all_ADC_measurement_values(&s_batt_pack_data);
  // ############# Einlesen der Temperatursensoren #############################
  u16_errors = read_temperatures_from_all_connected_sensors(&s_batt_pack_data);
  // Polling muss so häufig wie möglich durchgeführt werden um keine Botschaften zu verpassen
  poll_can_bus_single(&s_batt_pack_data);

  /* Neue Fehlerbits in lokale Batteriestruktur schreiben*/
  s_batt_pack_data.u16_error_bitfield |= u16_errors;
  
  #if(DEBUG_PRINT_ON)
  print_temperatures_from_all_connected_sensors(&s_batt_pack_data);
  #endif

  // ############# Fehlerbehandlung lokal fuer dieses Battery Pack #############################
  /* Warnung raushauen wenn die Batterie fast leer ist und der Luefter oder die Heizung aktiviert wurden.
  Dafuer das Statusbit nutzen TODO*/

  /* Hier auch plausibilitaetschecks durchfuehren wie z.B. Luefter aus aber trotzdem fliesst Strom - same with Heizung - */

  // ############# Sende gesammelte Daten über CAN #############################
  u16_errors = can_send_batt_pack_data(&s_batt_pack_data);
  // Reset aller error Bits aus dem letzten Zyklus
  u16_error_bitfield_after_can_send = 0u;
  /* Neue Fehlerbits zwischenspeichern*/
  u16_error_bitfield_after_can_send |= u16_errors;

  // ############# Fehlerbehandlung wenn CAN Kommunikation fehlschlaegt #############################
  /* Statusabfrage CAN SEND: Wenn keine Übertragung möglich TODO: Konzept überlegen
     1. Sofort die Daten versuchen über WIFI rauszusenden, damit man an die Fehlerbits kommt.
        Auf das ACK warten (mit einem Timeout) ob die Daten empfangen wurden.
        Wenn kein ACK kommt nach 1h erneut versuchen zu verbinden. Insgesamt 5 Versuche bis endgültig abgebrochen wird
        ueber WIFI - verbraucht sonst zuviel Strom. Ueber CAN wird einfach immer weiter versucht.
  */
  if(check_bit_16bit(u16_error_bitfield_after_can_send, ERROR_BIT_AT_LEAST_ONE_CAN_BEGIN_OR_END_FAILED) 
  || check_bit_16bit(u16_error_bitfield_after_can_send, ERROR_BIT_AT_LEAST_ONE_CAN_WRITE_FAILED)
  || check_bit_16bit(u16_error_bitfield_after_can_send, ERROR_BIT_CANNOT_CONNECT_TO_CAN))
  {
    // TODO Hier die Behandlung für den fehlgeschlagenen CAN einfuegen
  }

  // Wenn die Kommunikationsverbindung zum Master abbricht soll der Controller möglichst eigenständig seine Aufgaben durchführen
  // In dieser Funktion wird alles erledigt was dazu nötig ist
  autonomous_mode_handler();

  // Messung der Skriptlaufzeit ohne die Pollingschleife
  uint32_t u32_microseconds_end = micros();
  u32_micros_script_duration = u32_microseconds_end - u32_microseconds_start;

  // ############# POLLING - Empfange Daten über CAN (Polling ist notwendig da der D1 Mini kein SPI mit interrupt unterstützt) #############################
  #if(DEBUG_PRINT_ON)
  Serial.println("");
  Serial.print("Started Polling CAN BUS!");
  #endif
  poll_can_bus(&s_batt_pack_data);


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
  #endif

  #if(DEBUG_PRINT_ON)
  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("");
  #endif
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
  read_multi_temp_non_blocking_with_wait_time(TEMP_SENS_WAIT_TIME_NEXT_TEMP_READING_MILLIS);
  for(uint8_t u8_i=0u; u8_i < u8_temp_sens_device_count; u8_i++)
  {
    // Ueberpruefen auf Fehler und setzen der entsprechenden Fehlerbits
    if(fabs(af_temp_in_grad[u8_i] - (float_t)DEVICE_DISCONNECTED_C) < SMALL_FLOAT_VAL)
    {
        set_bit_16bit(&u16_status, ERROR_BIT_TEMP_SENS_DISCONNECTED);
        #if(DEBUG_PRINT_ON)
        Serial.println("");
        Serial.print("Temperature sensor is disconnected or broken.");
        #endif
    }
    else
    {
      /* No error occured - do nothing*/
    }
  }
  // Schreibe die Messwerte in die Datenhaltungsstruktur
  // TODO: Richtige Sortierung vor dem Einbau rausfinden und entsprechend beschriften
  ps_batt_pack_data->f_temp_air_cold_side     = af_temp_in_grad[0];            
  ps_batt_pack_data->f_temp_air_heating_side  = af_temp_in_grad[1];
  ps_batt_pack_data->f_temp_busbar_minus      = af_temp_in_grad[2];
  ps_batt_pack_data->f_temp_busbar_plus       = af_temp_in_grad[3];
  ps_batt_pack_data->f_temp_cable_shoe_1      = af_temp_in_grad[4];
  ps_batt_pack_data->f_temp_cable_shoe_2      = af_temp_in_grad[5];
  return u16_status;
}

#if(DEBUG_PRINT_ON)
void print_temperatures_from_all_connected_sensors(t_batt_pack_data *ps_batt_pack_data)
{
  for(uint8_t u8_i=0; u8_i < u8_temp_sens_device_count; u8_i++)
  {
    Serial.println("");
    Serial.print("Temp Sensor ");
    Serial.print((int)u8_i);
    Serial.print(": ");
    Serial.print(af_temp_in_grad[u8_i]);
    Serial.print("C");
  }
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
  // Polling muss so häufig wie möglich durchgeführt werden um keine Botschaften zu verpassen
  poll_can_bus_single(ps_batt_pack_data);

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
  // Polling muss so häufig wie möglich durchgeführt werden um keine Botschaften zu verpassen
  poll_can_bus_single(ps_batt_pack_data);

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
  // Polling muss so häufig wie möglich durchgeführt werden um keine Botschaften zu verpassen
  poll_can_bus_single(ps_batt_pack_data);
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
  size_t can_write_ret_val = 0xFF;
  int16_t i16_from_float;
  uint8_t au8_split_from_u16[2];

  // Paket 0 initialisieren, Werte berechnen, in das Paket schreiben und das Paket abschliessen
  i_can_begin_end_ret_val &= CAN.beginPacket(CAN_SEND_RASPI_ARBITRATION_ID_2BYTE_SUB_ID);
  if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_OK)
  {
    // Temperatursensor Kalte Seite
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_air_cold_side, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Temperatursensor beheizte Seite
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_air_heating_side, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Temperatursensor Busbar Minus
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_busbar_minus, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // 2 Byte Sub ID dieser Botschaft
    split_16bit_number_into_8bit_int16((int16_t)CAN_SEND_RASPI_2BYTE_SUB_ID_MSG_ID_TEMP1, au8_split_from_u16);
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

  // Polling muss so häufig wie möglich durchgeführt werden um keine Botschaften zu verpassen
  poll_can_bus_single(ps_batt_pack_data);

    // Paket 1 initialisieren, Werte berechnen, in das Paket schreiben und das Paket abschliessen
  i_can_begin_end_ret_val &= CAN.beginPacket(CAN_SEND_RASPI_ARBITRATION_ID_2BYTE_SUB_ID);
  if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_OK)
  {
    // Temperatursensor Busbar Plus
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_busbar_plus, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Temperatursensor Kabelschuh 1 des Verbindungskabels
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_cable_shoe_1, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Temperatursensor Kabelschuh 2 des Verbindungskabels
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_temp_cable_shoe_2, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // 2 Byte Sub ID dieser Botschaft
    split_16bit_number_into_8bit_int16((int16_t)CAN_SEND_RASPI_2BYTE_SUB_ID_MSG_ID_TEMP2, au8_split_from_u16);
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
    Serial.println("Paket 1 gesendet");
    #endif
  }

  // Polling muss so häufig wie möglich durchgeführt werden um keine Botschaften zu verpassen
  poll_can_bus_single(ps_batt_pack_data);

  // Paket 2 initialisieren, Werte berechnen, in das Paket schreiben und das Paket abschliessen
  i_can_begin_end_ret_val &= CAN.beginPacket(CAN_SEND_RASPI_ARBITRATION_ID_2BYTE_SUB_ID);
  if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_OK)
  {
    // Temperatursensor Busbar Plus
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_fan_current, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Temperatursensor Kabelschuh 1 des Verbindungskabels
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_heating_current, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // Temperatursensor Kabelschuh 2 des Verbindungskabels
    i16_from_float = float_to_fixed(ps_batt_pack_data->f_voltage_sens_battery_pack, FIXED_POINT_FRACTIONAL_BITS_MAX128);
    split_16bit_number_into_8bit_int16(i16_from_float, au8_split_from_u16);
    can_write_ret_val &= CAN.write(au8_split_from_u16[0]);
    can_write_ret_val &= CAN.write(au8_split_from_u16[1]);
    #if(DEBUG_PRINT_ON)
    Serial.print(au8_split_from_u16[0], HEX);
    Serial.print(" ");
    Serial.print(au8_split_from_u16[1], HEX);
    Serial.print(" ");
    #endif
    // 2 Byte Sub ID dieser Botschaft
    split_16bit_number_into_8bit_int16((int16_t)CAN_SEND_RASPI_2BYTE_SUB_ID_MSG_ID_CURRENT, au8_split_from_u16);
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
    Serial.println("Paket 2 gesendet");
    #endif
  }

  // Polling muss so häufig wie möglich durchgeführt werden um keine Botschaften zu verpassen
  poll_can_bus_single(ps_batt_pack_data);

  // Paket 1 initialisieren, Werte berechnen, in das Paket schreiben und das Paket abschliessen
  i_can_begin_end_ret_val &= CAN.beginPacket(CAN_SEND_RASPI_ARBITRATION_ID_2BYTE_SUB_ID);
  if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_OK)
  {
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
      set_bit_16bit(&(ps_batt_pack_data->u16_error_bitfield), ERROR_BIT_AT_LEAST_ONE_CAN_BEGIN_OR_END_FAILED);
    }
    if(can_write_ret_val == CAN_LIB_RET_VAL_ERROR)
    {
      set_bit_16bit(&(ps_batt_pack_data->u16_error_bitfield), ERROR_BIT_AT_LEAST_ONE_CAN_WRITE_FAILED);
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
    Serial.println("Paket 3 gesendet");
    #endif
  }

  // Polling muss so häufig wie möglich durchgeführt werden um keine Botschaften zu verpassen
  poll_can_bus_single(ps_batt_pack_data);

  // Setzen der Fehlerbits der erkannten Fehler
  if(i_can_begin_end_ret_val == CAN_LIB_RET_VAL_ERROR)
  {
    set_bit_16bit(&u16_status, ERROR_BIT_AT_LEAST_ONE_CAN_BEGIN_OR_END_FAILED);
  }
  if(can_write_ret_val == CAN_LIB_RET_VAL_ERROR)
  {
    set_bit_16bit(&u16_status, ERROR_BIT_AT_LEAST_ONE_CAN_WRITE_FAILED);
  }

  #if(DEBUG_PRINT_ON)
  Serial.print("");
  Serial.print("Fehlerbitfeld lokale CAN Send Funktion: ");
  Serial.print(u16_status);
  Serial.print("");
  #endif

  return u16_status;
}

uint16_t create_can_id_from_device_id(uint8_t u8_device_id)
{
  // Dokumentation zur Berechnung und Logik dahinter in Libre Office Dokument Smarthome_CAN_Doku.ods
  uint16_t u16_temp = (uint16_t)u8_device_id;
  return (u16_temp<<3);
}

void read_multi_temp_non_blocking_with_wait_time(uint32_t u32_wait_time_millis)
{
  for(uint8_t u8_i=0u; u8_i < u8_temp_sens_device_count; u8_i++)
  {
    // Wenn die eingestellte Wartezeit für die nächste Temperaturmessung abgelaufen ist
    if ( (millis() - u32_last_temp_read_time[u8_i]) > u32_wait_time_millis)
    {
      // Zeitüberprüfung ob die Konversionszeit von ~750ms abgelaufen ist
      if ( (millis() - u32_last_temp_read_time[u8_i]) > u32_temp_conversiontime)
      {
        af_temp_in_grad[u8_i] = temp_sensors.getTempC(au8_temp_sens_addr[u8_i]);
        temp_sensors.requestTemperatures();                    // ask for next reading 
        u32_last_temp_read_time[u8_i] = millis();   
      }
    }
  }
}

void poll_can_bus(t_batt_pack_data *ps_batt_pack_data)
{
  uint32_t u32_microseconds_polling_start = micros();
  uint32_t u32_microseconds_polling_end;
  for(int i=0; i < NOF_MILLISECONDS_WAIT_TIME_FOR_NEXT_CYCLE;i++)
  {
    if((i % RECEIVE_POLLING_NOF_SKIPPED_WAIT_MILLISECONDS) == 0)
    {
      poll_can_bus_single(ps_batt_pack_data);
    }
    delay(1);
    // Sicherstellen das die Warteschleife nicht viel länger läuft als die eingestellte Anzahl an Millisekunden
    u32_microseconds_polling_end = micros();
    uint32_t u32_time_diff = (u32_microseconds_polling_end - u32_microseconds_polling_start);
    if(u32_time_diff > (NOF_MILLISECONDS_WAIT_TIME_FOR_NEXT_CYCLE * 1000))
    {
      #if(DEBUG_PRINT_ON)
      Serial.println("");
      Serial.print("Wartezeit in µs: ");
      Serial.print(u32_time_diff);
      #endif

      break;
    }
  }
}

void poll_can_bus_single(t_batt_pack_data *ps_batt_pack_data)
{
  // Start Polling MPC2515 if it received CAN Messages
  uint8_t u8_nof_received_bytes = CAN.pollCANData(au8_rx_buffer, &u16_rx_can_ids, INPUT_BUFFER_SIZE_IN_BYTES);

  // Wenn jetzt noch Bytes vorhanden sind, war der Empfangspuffer in diesem Skript zu klein --> Fehler !!!
  if (CAN.available())
  {
    // Fehlermeldung: Möglicher Datenverlust auf dem CAN wegen zu kleinem Empfangsspeicher
    set_bit_16bit(&u16_error_bitfield_after_can_send, ERROR_BIT_POSSIBLE_DATA_LOSS_CAN);

    #if (DEBUG_PRINT_ON)
    Serial.println("");
    Serial.print("Possible data loss on CAN due to too small receive buffer.");
    #endif
  }
  // Wenn eine Botschaft empfangen wurde starte das Handling der Botschaft
  if (u8_nof_received_bytes > 0u)
  {
    #if (DEBUG_PRINT_ON)
    Serial.println("");
    Serial.print("Number of received bytes: ");
    Serial.print(u8_nof_received_bytes);
    Serial.println("");
    for (uint8_t i = 0; i < u8_nof_received_bytes; i++)
    {
      Serial.print(au8_rx_buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println("");
    Serial.print("ID of received bytes: ");
    Serial.println("");
    Serial.print(u16_rx_can_ids);
    #endif
    // Wenn die Botschaft eine 2 Byte Sub ID hat...
    if (u16_rx_can_ids == CAN_REC_ARBITRATION_ID_2BYTE_SUB_ID)
    {
      // ...ermittle welche Botschaft das ist
      // interpretiere die 2 Sub ID Bytes als eine 16 Byte Zahl
      uint16_t u16_two_byte_sub_id = merge_8bit_numbers_into_16bit_uint16(&au8_rx_buffer[6]);
      #if (DEBUG_PRINT_ON)
      Serial.println("");
      Serial.print("Received two byte sub id: ");
      Serial.print(u16_two_byte_sub_id);
      #endif
      // Wenn Globaltime Botschaft...
      if (u16_two_byte_sub_id == CAN_REC_2BYTE_SUB_ID_MSG_ID_GLOBALTIME)
      {
        // Stelle die Uhr auf die empfangene Zeit/Datum
        setTime((int)au8_rx_buffer[3], (int)au8_rx_buffer[4], (int)au8_rx_buffer[5], (int)au8_rx_buffer[2], (int)au8_rx_buffer[1], (int)au8_rx_buffer[0] - 30);
        // Eine neue Uhrzeit wurde Empfanger - Raspi Smarthome Master lebt also noch
        u32_last_master_alive_seen_time = millis();
        #if DEBUG_PRINT_ON
        Serial.println("");
        Serial.print("Tag: ");
        Serial.print(day());
        Serial.print("   Monat: ");
        Serial.print(month());
        Serial.print("   Jahr: ");
        Serial.print(year());
        Serial.println("");
        Serial.print("Stunde: ");
        Serial.print(hour());
        Serial.print("   Minute: ");
        Serial.print(minute());
        Serial.print("   Sekunde: ");
        Serial.print(second());
        #endif
      }
      else
      {
        set_bit_16bit(&(ps_batt_pack_data->u16_error_bitfield), ERROR_BIT_UNKOWN_2BYTE_SUB_ID);
        #if (DEBUG_PRINT_ON)
        Serial.println("");
        Serial.print("ERROR: Unknown two byte sub id: ");
        Serial.print(u16_two_byte_sub_id);
        #endif
      }
    }
  }
}


void autonomous_mode_handler()
{
  uint32_t u32_current_time_millis = millis();
  uint32_t u32_time_diff = (u32_current_time_millis - u32_last_master_alive_seen_time);

  // Wenn die Verbindung zum Master abgebrochen ist setze das entsprechende Flag
  if((u32_time_diff >= MILLISECONDS_FOR_AUTONOMOUS_MODE_WHEN_MISSING_MASTER) && (b_autonomous_mode == false))
  {
    #if DEBUG_PRINT_ON
    Serial.println("Autonomer Modus...ECU versucht alle Aufgaben ohne Output von Aussen zu regeln...");
    #endif
    b_autonomous_mode = true;
    // TODO: WLAN Verbindung und MQTT Verbindung herstellen

  }
  
  // Flag Reset
  if((u32_time_diff < MILLISECONDS_FOR_AUTONOMOUS_MODE_WHEN_MISSING_MASTER) && (b_autonomous_mode == true))
  {
    #if DEBUG_PRINT_ON
    Serial.println("Autonomer Modus aus.");
    #endif
    b_autonomous_mode = false;
    // TODO: WLAN Verbindung und MQTT Verbindung wieder kappen
  }
}

