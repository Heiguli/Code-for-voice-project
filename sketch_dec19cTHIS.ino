#include <Arduino.h>
#include <I2S.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoMqttClient.h>
#include <voice_dig_inferencing.h>
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

//^ the needed libraries included 

#define LED_PIN LED_BUILTIN //Defining led

// -------------------- AUDIO --------------------
#define SAMPLE_RATE   16000 //Sample rate
#define BLOCK_SIZE    512   //Size of the blocks of audio data that are tested for rms
#define RECORD_SECONDS 2    //how long one actual recording is
#define NUM_SAMPLES   (SAMPLE_RATE * RECORD_SECONDS) //Number of data samples in the actual recorded audio
#define PRE_BLOCKS    4     //The amount of preblocks needed to pass rms. 
#define THRESHOLD     1000  //Rms value threshold

int16_t samples_for_block[BLOCK_SIZE]; //data place for blocks. Blcoks are used to test rms
int16_t samples_for_record[NUM_SAMPLES];  //data place for actual recorded audio that was recorded/turned into wav files and used for model training in edge impulse
int16_t pre_buffer[PRE_BLOCKS * BLOCK_SIZE];  //Prebuffer to hold the blocks of audio data that rms were being checked. If rms over threshold - save the prebuffer to samples_for_record
int pre_index = 0; //
int count_value = 0; //Just a index to cound how many blocks have had rms over threshold continously



// -------------------- WIFI / MQTT --------------------
const char* ssid     = "Honor8.12"; //ssid of the mobile hotspot used to connect to internet
const char* password = "pineapple1234"; //password of the wifi hotspot

const char* mqtt_server = "1e1002fc03794c068156c38e97f53e12.s1.eu.hivemq.cloud"; //hivemq server link
const int   mqtt_port   = 8883; //port used
const char* mqtt_user   = "ananas1234"; //hivemq credentials username
const char* mqtt_pass   = "Password1234"; //hivemq credentials password
const char* mqtt_topic  = "voice/commands"; ////hivemq topic

WiFiClientSecure net;
MqttClient mqtt(net);

// -------------------- I2S --------------------
I2S i2s(INPUT); //Making the I2S object to be able to use the methods from that library

// -------------------- HELPERS --------------------
void read_audio_block(int16_t *buffer, int n) {  //Method to read a audio block of size of n
    int16_t left, right; //int_16 size int value, left and right
    for (int i = 0; i < n; i++) {  //we go through the whole array
        if (i2s.read16(&left, &right)) buffer[i] = left; //If there are values, we only take left one and add it to the array. 
                                                        //We used a digital microhpone with two lane data output. We only needed the first value because the second value was a dublicate of the first value
        else buffer[i] = 0;
    }
}

float count_rms(int16_t *block, int n) {  //Method to count the rms of a said block of audio data. 
    float mean = 0; 
    for (int i = 0; i < n; i++) mean += block[i];
    mean /= n;

    float acc = 0;
    for (int i = 0; i < n; i++) {
        float d = block[i] - mean;
        acc += d * d;
    }
    return sqrt(acc / n);
}

void record_audio_with_prebuffer() { //Audio recording with prebuffer, so we take t
    int idx = 0;
    for (int i = 0; i < PRE_BLOCKS * BLOCK_SIZE && idx < NUM_SAMPLES; i++)
        samples_for_record[idx++] = pre_buffer[i];

    int16_t left, right;
    while (idx < NUM_SAMPLES) {
        if (i2s.read16(&left, &right)) samples_for_record[idx++] = left;
        else samples_for_record[idx++] = 0;
    }
}

// Edge Impulse: convert int16 to float
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) {
    numpy::int16_to_float(&samples_for_record[offset], out_ptr, length);
    return 0;
}

// -------------------- WIFI / MQTT --------------------
void setup_wifi() {  //Method to setup the wifi
    Serial.print("Connecting WiFi"); //Just print function to make 
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" connected!");
}

void connect_mqtt() {
    net.setInsecure(); // skip TLS verification for simplicity
    mqtt.setUsernamePassword(mqtt_user, mqtt_pass);

    Serial.print("Connecting MQTT...");
    while (!mqtt.connect(mqtt_server, mqtt_port)) {
        Serial.print(" failed, rc=");
        Serial.println(mqtt.connectError());
        delay(2000);
    }
    Serial.println(" connected!");
}

// -------------------- SETUP --------------------
void setup() { //setup method where pins are set
  unsigned long fullStartTime = millis(); 
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    i2s.setBCLK(10);
    i2s.setDIN(12);
    i2s.begin(SAMPLE_RATE);

    setup_wifi();
    connect_mqtt();

    run_classifier_init();

    Serial.println("System ready");
    unsigned long fullEndTime = millis();
    unsigned long fullTime = fullEndTime - fullStartTime;
    Serial.print(fullTime);
}

// -------------------- LOOP --------------------
void loop() {
    unsigned long startTime = millis();
    mqtt.poll();  // keep MQTT alive

    read_audio_block(samples_for_block, BLOCK_SIZE);
    float rms = count_rms(samples_for_block, BLOCK_SIZE);

    if (rms > THRESHOLD) {
        memcpy(&pre_buffer[pre_index * BLOCK_SIZE],
               samples_for_block,
               BLOCK_SIZE * sizeof(int16_t));

        pre_index = (pre_index + 1) % PRE_BLOCKS;
        count_value++;

        if (count_value >= PRE_BLOCKS) {
            record_audio_with_prebuffer();
            signal_t signal;
            signal.total_length = NUM_SAMPLES;
            signal.get_data = microphone_audio_signal_get_data;
           // unsigned long startTime = millis();
            ei_impulse_result_t result = {0};
            if (run_classifier_continuous(&signal, &result, false) == EI_IMPULSE_OK) {
                for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                    float v = result.classification[i].value;
                    const char* lbl = result.classification[i].label;

                    // LED control
                    if (v > 0.7f) {
                        if (strcmp(lbl, "on") == 0) {
                          digitalWrite(LED_PIN, HIGH);
                        }
                        if (strcmp(lbl, "off") == 0) {
                          digitalWrite(LED_PIN, LOW);
                        }

                        // Publish to HiveMQ
                        mqtt.beginMessage(mqtt_topic);
                        mqtt.print("{\"label\":\"");
                        mqtt.print(lbl);
                        mqtt.print("\",\"confidence\":");
                        mqtt.print(v, 3);
                        mqtt.print("}");
                        mqtt.endMessage();
                        unsigned long endTime = millis();
                        unsigned long time = endTime - startTime;
                        Serial.print(time);

                        Serial.print("Published: ");
                        Serial.println(lbl);

                    }
                }
            }
            count_value = 0;
        }
    } else {
        count_value = 0;
    }
}