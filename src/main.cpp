#include <scheduler.h>
#include <ArduinoJson.h>
#include "TDS.h"
#include "Temperature.h"

//---WiFi & MQTT---
#include <WiFi.h>
#include <PubSubClient.h>
#define WIFI_SSID "Hehe"			  //"nama_wifi"
#define WIFI_PASSWORD "apahayoo"	  //"password_wifi"
#define MQTT_SERVER "broker.mqtt.com" //"broker.mqtt.com"
#define MQTT_PORT 1883
#define MQTT_USERNAME "apaaja"	 //"username_mqtt"
#define MQTT_PASSWORD "verovero" //"password_mqtt"
#define MQTT_TOPIC "/athus/data"
WiFiClient espClient;
PubSubClient client(espClient);

//---TEMP---
TemperatureDallas DS18B;
float tempC, tempF, rawTemp, correctedTemp;
float rawHigh = 99.6;
float rawLow = 0.5;
float referenceHigh = 99.9;
float referenceLow = 0;
float rawRange = rawHigh - rawLow;
float referenceRange = referenceHigh - referenceLow;

//---TDS---
GravityTDS tds;
float baseTemperature;
float tdsValue = 0;

//---tvvtgvgt---
unsigned char data[4] = {};
float distance, filteredDistance, curDistance, waterLevel;
float height = 100;
float alpha = 0.2;

//---pH---
#define SensorPin 34   // pH meter Analog output to Arduino Analog Input 0
#define Offset 0.00	   // deviation compensate
#define ArrayLength 40 // times of collection

int pHArray[ArrayLength]; // Store the average value of the sensor feedback
int pHArrayIndex = 0;
float pHValue, voltage;

//---Turbidity---
#define TurbyPin 35
int ntu_result;
float turby_voltage;

StaticJsonDocument<200> json_data;
String message = "null";

//---gps---
float latitude, longitude;

double avergearray(int *arr, int number);
void read_TEMP();
void read_US();
void read_pH();
void read_TDS();
void read_Turby();
void reconnect_MQTT();
void print_Data();
#include <HardwareSerial.h>

// Define the serial port to use for GPS communication
HardwareSerial tvvtgvgt(2);
void setup()
{
	Serial.begin(115200);
	tvvtgvgt.begin(9600);

	tds.begin();

	// DS18B Temperature Sensor Initialization. This magic number is responsible
	// of the error code returned by the DS18B.begin() method.
	if (DS18B.begin() == -1000)
		Serial.println("Sensor not found!");
	else if (DS18B.begin() == -2000)
		Serial.println("CRC is not valid!");
	else if (DS18B.begin() == -3000)
		Serial.println("Device is not recognized!");

	// WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
	// while (WiFi.status() != WL_CONNECTED) {
	//   Serial.println("Connecting to WiFi...");
	// }
	// Serial.println("CONNECTED");
	// client.setServer(MQTT_SERVER, MQTT_PORT);

	// SENSOR1_TASK sensor1(20);
	// SENSOR2_TASK sensor2(200);
	// // MQTT_TASK mqtt_task(5000);
	// LOG_TASK logger(1000);

	// Task *tasks[] = {&sensor1, &logger, /*&mqtt_task*/};
	// Task *tasks[] = {
	//     &sensor1,
	//     &sensor2,
	//     &logger
	//     // &mqtt_task,
	// };
	// Scheduler scheduler(tasks, NUM_TASKS(tasks));
	// while (1)
	// {
	//   scheduler.runTasks();
	// }
}

void loop()
{
	read_TEMP();
	read_US();
	delay(150);
	read_pH();
	read_TDS();
	read_Turby();
	print_Data();
}

void read_TEMP()
{
	rawTemp = DS18B.getTemperature();
	correctedTemp = (rawTemp - rawLow) / rawRange * referenceRange + referenceLow;
	delay(100);
}

void read_TDS()
{
	if (correctedTemp < 50)
		tds.setReferenceTemperature(correctedTemp);
	else
		tds.setReferenceTemperature(25);
	tds.updateTDS();
	tdsValue = tds.readTDS();
	delay(100);
}

double avergearray(int *arr, int number)
{
	int i;
	int max, min;
	double avg;
	long amount = 0;
	if (number <= 0)
	{
		Serial.println("Error number for the array to avraging!/n");
		return 0;
	}
	if (number < 5)
	{ // less than 5, calculated directly statistics
		for (i = 0; i < number; i++)
		{
			amount += arr[i];
		}
		avg = amount / number;
		return avg;
	}
	else
	{
		if (arr[0] < arr[1])
		{
			min = arr[0];
			max = arr[1];
		}
		else
		{
			min = arr[1];
			max = arr[0];
		}
		for (i = 2; i < number; i++)
		{
			if (arr[i] < min)
			{
				amount += min; // arr<min
				min = arr[i];
			}
			else
			{
				if (arr[i] > max)
				{
					amount += max; // arr>max
					max = arr[i];
				}
				else
				{
					amount += arr[i]; // min<=arr<=max
				}
			} // if
		}	  // for
		avg = (double)amount / (number - 2);
	} // if
	return avg;
}

// Reconnecting to MQTT
void reconnect_MQTT()
{
	while (!client.connected())
	{
		// Serial.println("Connecting to MQTT...");
		if (client.connect("ESP32Client", MQTT_USERNAME, MQTT_PASSWORD))
		{
			// Serial.println("Connected to MQTT");
		}
		else
		{
			Serial.print("Failed to connect to MQTT, rc=");
			Serial.println(client.state());
		}
	}
}

void read_US()
{
  int receivedByte;
  do {
    for (int i = 0; i < 4; i++)
    {
      data[i] = receivedByte = tvvtgvgt.read();
    }
  } while (receivedByte == 0xff);

  tvvtgvgt.flush();

  if (data[0] == 0xff)
  {
    int sum;
    sum = (data[0] + data[1] + data[2]) & 0x00FF;
    if (sum == data[3])
    {
      distance = (data[1] << 8) + data[2];
	  distance = distance/10;
      if (distance > 280)
      {
        Serial.print("distance=");
        Serial.print(distance / 10);
        Serial.println("cm");
      } else
      {
        Serial.println("Below the lower limit");
      }
    } else Serial.println("ERROR");
  }
}


void read_pH()
{
	pHArray[pHArrayIndex++] = analogRead(SensorPin);
	if (pHArrayIndex == ArrayLength)
		pHArrayIndex = 0;
	voltage = avergearray(pHArray, ArrayLength) * 5.0 / 1024;
	pHValue = 3.5 * voltage + Offset;
}

void read_Turby()
{
	turby_voltage = analogRead(TurbyPin) * 5 / 4095.0;
	// ntu_result = -2572.2 * (turby_voltage * turby_voltage) + 8700.5 * turby_voltage - 4352.9;
	// ntu_result = 2572.2 * (turby_voltage * turby_voltage) + 8700.5 * turby_voltage + 4352.9;
	ntu_result = map(turby_voltage, 0, 4, 500, 5);
	// got from substitut 3 diferent voltage reading on 3 different condition of water
	//  into this eqwqeq :
	//  y = -ax² + bx - c
	//  then get a, b, and c
}

void print_Data()
{
	json_data["name"] = "BengawanSolo-1";
	// json_data["latitude"] = float(latitude);
	// json_data["longitude"] = float(longitude);
	json_data["waterLevel"] = float(distance);
	json_data["kekeruhan"] = float(tdsValue);
	json_data["pH"] = float(pHValue);
	json_data["suhuAir"] = float(correctedTemp);
	json_data["turbidity"]= float(ntu_result);
	Serial.println("");
	serializeJson(json_data, Serial);

	/*
		name : String (BengawanSolo-0)
	  latitude : float
	  longitude : float
	  waterLevel : float
	  pH : float
	  kekeruhan (ini ntu) : float
	  suhuAir : float*/

	// Serial.println(ntu_result);
}