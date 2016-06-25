#include <Arduino.h>
#include "Logging.h"
#include "SimpleTimer.h"
#include "OneWire.h"
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

void start_sonar();
int get_sonar_baseline(int variance);
int get_sonar_range();
void update_sonar();
void check_sonar_baseline();

void start_wifi();
void upload_data();
void assemble_data_packet(char* packet_buffer);
void add_to_array(char* buffer, char* insert);

const int BASELINE_VARIANCE_THRESHOLD = 20;
const int CHECK_RANGE_INTERVAL = 100;
const int RANGE_DETECT_THRESHOLD = 70; // Minimum threshold for range detection in cm. (Target must move at least this much)
const byte MIN_BASELINE_READS = 20; // Minimum number of reads needed to establish a baseline
const byte MAX_BASELINE_READS = 30; // Maximum number of reads to establishe a baseline
enum SONAR_READ_METHODS{
    PULSE = 1,
    ANALOG = 2
};
const int SONAR_READ_METHOD = PULSE;
const int BASELINE_READ_INTERVAL = 200; // Time between range baseline calibration reads in milliseconds
const byte MIN_SUCCESSIVE_SONAR_READS = 0;  // Number of consecutive sonar reads that will result in a 'detection'

const byte TEMPERATURE_RESOLUTION = 12; //12-bit temperature resolution
const long PRINT_INTERVAL = 60000;
const int CHECK_ENVIRONMENTAL_SENSOR_INTERVAL = PRINT_INTERVAL/3;

const byte SONAR_PIN = D1;

const char* DEVICE_NAME = "PingTrip";

const char* SSID = "Totally-legit-network";
const char* PASSWORD = "";

const char* SERVER_ADDRESS = "www.dweet.io";
const int SERVER_PORT = 80;
const char GET_REQUEST[] = "GET /dweet/for/";
const char GET_TAIL[] = " HTTP/1.1\r\n\r\n";
const long TIMEOUT = 5000;  // TCP timeout in ms
///////////////////////////////////////////////////////////////////////////////
// Globals (because I'm shit like that)

int sonar_baseline = 0;
int sonar_range = 0;
long sonar_count = 0;
int sonar_average = 0;
int successive_sonar_detections = 0;
long last_sonar_count = 0;


float air_temperature = 0;

SimpleTimer timer;

///////////////////////////////////////////////////////////////////////////////
// Main
void setup(){
    Log.Init(LOG_LEVEL_VERBOSE, 57600);
    Log.Info("Starting People Counter...");

    start_sonar();
    start_wifi();
}

void loop(){
    timer.run();
}


///////////////////////////////////////////////////////////////////////////////
/* Sonar */
void start_sonar(){
	/**
	* Start the sonar/ultrasonic rangefinder
	* A baseline range (to the ground or static surrounds) is established for comparing against new measurements.
	* The sonar is disabled if the sensor cannot establish a baseline measurement
	*/

	Log.Debug("Ultrasonic - Establishing baseline range...");
	sonar_baseline = get_sonar_baseline(BASELINE_VARIANCE_THRESHOLD);

	sonar_count = 0;
	sonar_range = 0;
	timer.setInterval(CHECK_RANGE_INTERVAL, update_sonar);
    timer.setInterval(2000, check_sonar_baseline);

	// Enable the sensor if it passes the baseline check
	if (sonar_baseline > RANGE_DETECT_THRESHOLD) {
		Log.Info("Sonar started - Baseline: %dcm", sonar_baseline);
	}else{
		sonar_baseline = 0;
		Log.Error("Sonar initialisation failed - sensor disabled");
	}

	update_sonar();
}


int get_sonar_baseline(int variance){
	/**
	* Establish the baseline range from the sensor to the ground
	* The sensor will take samples until the readings are consistent or until timeout
	* :variance: Maximum amount of variance allowed between reads for valid baseline (in cm)
	* :return: Average variance of the baseline in cm
	*/
	int average_range = get_sonar_range();
	int baseline_reads = 1;
	int average_variance = 500;

	// Keep reading in the baseline until it stablises
	while ((baseline_reads < MIN_BASELINE_READS || average_variance > variance) && baseline_reads < MAX_BASELINE_READS){
		int new_range = get_sonar_range();
		int new_variance = abs(average_range - new_range);

		average_variance = ((average_variance + new_variance) / 2);
		average_range = ((average_range + new_range) / 2);

		Log.Debug("Ultrasonic Calibration: Range - %d, Variance - %d", int(average_range), average_variance);

		baseline_reads++;
		delay(BASELINE_READ_INTERVAL);
	}

	// Baseline could not be established; send invalid measurement
	if (average_variance > variance) {
		average_range = -1;
	}

	return average_range;
}


int get_sonar_range(){
	/**
	* Get the range in cm from the ultrasonic rangefinder
	* :return: Distance to object in cm
	*/

	long raw_time_of_flight = 0;

	if (SONAR_READ_METHOD == PULSE){
		// The time of flight for the entire pulse, including the transmitted and reflected wave.
		raw_time_of_flight = pulseIn(SONAR_PIN, HIGH);
	}

	else if (SONAR_READ_METHOD == ANALOG) {
		// Time of flight = uncompensated distance * 58us
		// Uncompensated distance = analog read * 2cm
		raw_time_of_flight = analogRead(SONAR_PIN) * 58 * 2;
	}


	// Ref: https://en.wikipedia.org/wiki/Speed_of_sound#Practical_formula_for_dry_air
	// Humidity does affect the speed of sound in air, but only slightly
	long compensated_distance = (raw_time_of_flight * (331.3 + 0.606 * 25)) / 20000;

	Log.Verbose("Ultrasonic Range: %d cm", compensated_distance);

	return int(compensated_distance);
}


void update_sonar(){
	/**
	* Check the ultrasonic range sensor to see if a traffic event has occurred.
	* Traffic events are counted as a break in the sensor's 'view' of the ground.
	* Any object between the sensor and the ground baseline will cause the sensor to register a shorter range than usual.
	*/
	sonar_range = get_sonar_range();

	// Detection occurs when target breaks the LoS to the baseline
	if ((sonar_baseline - sonar_range) > RANGE_DETECT_THRESHOLD){

		// If n in a row
		if (successive_sonar_detections == MIN_SUCCESSIVE_SONAR_READS){
			successive_sonar_detections += 1;

			// Increase traffic count
			sonar_count++;
			Log.Info("Traffic count - Sonar: %d counts", sonar_count);
		}

		else if (successive_sonar_detections < MIN_SUCCESSIVE_SONAR_READS){
			successive_sonar_detections += 1;
		}
	}
	else{
		successive_sonar_detections = 0;
	}

    sonar_average = (sonar_average + sonar_range)/2;
}


void check_sonar_baseline(){
    int counts_since_last_check = sonar_count - last_sonar_count;

    // Check if baseline needs to be recalculated
    if ((sonar_average - sonar_baseline) > BASELINE_VARIANCE_THRESHOLD){
        sonar_baseline = sonar_average;
    }

    if (counts_since_last_check == 0 and (sonar_baseline - sonar_average) > BASELINE_VARIANCE_THRESHOLD)
        sonar_baseline = sonar_average;

    sonar_average = sonar_range;
    last_sonar_count = sonar_count;
}

///////////////////////////////////////////////////////////////////////////////
// Wifi
void start_wifi(){
    Log.Info("Connecting to %s", SSID);
    WiFi.begin(SSID, PASSWORD);

    // Wait for connection before continuing
    while (WiFi.status() != WL_CONNECTED) {
       delay(500);
       Serial.print(".");
    }

    Log.Info("WiFi Connected!\nIP address: %s", WiFi.localIP().toString().c_str());
    timer.setInterval(5000, upload_data);
}


void upload_data(){
    // Use WiFiClient class to create TCP connections
    WiFiClient client;
    char packet_buffer[200];
    bool connection_successful = true;

    // Attempt connection to data server
    if (!client.connect(SERVER_ADDRESS, SERVER_PORT)) {
        Log.Error("Connection to [%s] failed", SERVER_ADDRESS);
        connection_successful = false;
    }

    // Send data if connection is available
    if (connection_successful){
        assemble_data_packet(packet_buffer);
        client.print(packet_buffer);

        delay(10);
        while (client.available() > 0){
            Serial.print(client.readString());
            delay(10);
        }

        client.stop();
    }
}


void assemble_data_packet(char* packet_buffer){

    char entry[100];

    sprintf(packet_buffer, "%s%s?", GET_REQUEST, DEVICE_NAME);

    sprintf(entry, "&sonar_count=%d&sonar_average=%d&sonar_baseline=%d", sonar_count, sonar_average, sonar_baseline);
    add_to_array(packet_buffer, entry);

    add_to_array(packet_buffer, const_cast<char*>(GET_TAIL));
}


void add_to_array(char* buffer, char* insert){
    int index = strlen(buffer);

    for (int i = 0; i < strlen(insert); i++) {
        buffer[index + i] = insert[i];
    }

    buffer[index + strlen(insert)] = '\0';
}
