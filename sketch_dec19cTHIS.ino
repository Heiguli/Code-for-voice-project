#include <Arduino.h> //Core arduino library
#include <I2S.h> //Library for I2S digital microphones
#include <math.h> //Core math library
#include <WiFi.h> //Enables wifi library 
#include <WiFiClientSecure.h> //Enables secure wifi connection (TLS)
#include <ArduinoMqttClient.h> //Mqtt library to connect to mqtt and to paste messages yms.
#include <voice_dig_inferencing.h> //Include the edge impulse trained model
#include "edge-impulse-sdk/classifier/ei_run_classifier.h" //Edge impulse inference engine, the confidence score on on / ooff

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

WiFiClientSecure net; //Creates a secure network object from WiFiClientSecure
MqttClient mqtt(net); //uses the secure network object to send messages to mqtt

// -------------------- I2S --------------------
I2S i2s(INPUT); //Making the I2S object to be able to use the methods from that library

// -------------------- HELPERS --------------------
void read_audio_block(int16_t *buffer, int n) {  //Method to read a audio block of size n
    int16_t left, right; //int_16 int value, left and right for two audio channel values
    for (int i = 0; i < n; i++) {  //we go through the whole array, n parameter is put in as the block size, when the method is called, 
        if (i2s.read16(&left, &right)) buffer[i] = left; //Use I2S library method read16, If there are values, we only take left one and add it to the array. 
                                                        //We used a digital microhpone with two lane data output. We only needed the first value because the second value was a dublicate
        else buffer[i] = 0; //Ignore the dublicate right value
    }
}

float count_rms(int16_t *block, int n) {  //Method to count the rms of a said block "int16_t block" of audio data. 
    float mean = 0; //For storing the mean value of the audio samples from the block
    for (int i = 0; i < n; i++) {
        mean += block[i]; //Sum all the values from block together. 
        }
    mean /= n; //Calculate the mean of audio values

    float acc = 0; //Accumulator variable 
    for (int i = 0; i < n; i++) { 
        float d = block[i] - mean; //Centers the signal/values around zero.
        acc += d * d;  //remove negative values and adds the squared value to the accumulator
    }
    return sqrt(acc / n); //return rms of the block
}

void record_audio_with_prebuffer() { //Audio recording with prebuffer, meaning saving audio data after + before rms trigger. 
    int idx = 0; //Just index to keep track of position in the last loop in this method
    for (int i = 0; i < PRE_BLOCKS * BLOCK_SIZE && idx < NUM_SAMPLES; i++) //We copy the contents from the prebuffer 
        samples_for_record[idx++] = pre_buffer[i]; //to the actual save place samples_for_record

    int16_t left, right;
    while (idx < NUM_SAMPLES) { //Then the same stuff as in block method, 
        if (i2s.read16(&left, &right)) samples_for_record[idx++] = left; //only save left audio channel value
        else samples_for_record[idx++] = 0;
    }
}

//Method used by edge impulse to access audio data
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr) { //offset = starting point, lenght = how many samples, out_ptr = pointer where data will be written
    numpy::int16_to_float(&samples_for_record[offset], out_ptr, length); //converts audio samples from 16 bit to floating
    return 0; //return zero for succesfull execution of the function
}

// -------------------- WIFI / MQTT --------------------
void setup_wifi() {  //Method to setup the wifi
    Serial.print("Connecting WiFi"); //Just print function to make thigns more clear
    WiFi.begin(ssid, password); //start the connection with presaid ssid, password
    while (WiFi.status() != WL_CONNECTED) {  //Wait for connection, after every 0.5 seconds, print "." The whole program is paused on this if no connection.  
        delay(500);
        Serial.print(".");
    }
    Serial.println(" connected!"); //Println for clearer program execution
}

void connect_mqtt() {
    net.setInsecure(); // skip TLS verification for simplicity
    mqtt.setUsernamePassword(mqtt_user, mqtt_pass);  //setting the username and password of mqtt client. Mqtt broker needs these

    Serial.print("Connecting MQTT..."); //Print status for more clear code
    while (!mqtt.connect(mqtt_server, mqtt_port)) { //trying to connect to mqtt server with predetermined ports/server links
        Serial.print(" failed, rc="); //Failed with-
        Serial.println(mqtt.connectError()); //-connection error
        delay(2000);
    }
    Serial.println(" connected!"); //print statement to make code more clean
}

// -------------------- SETUP --------------------
void setup() { //setup method where pins and wifi setup
    Serial.begin(115200);//serial communication with 115200 bauds
    pinMode(LED_PIN, OUTPUT); //build in pin as output

    i2s.setBCLK(10); //Set I2S digital microphone pins, clock   
    i2s.setDIN(12); //data input
    i2s.begin(SAMPLE_RATE); //starts the I2S interface with sample rate (16000)

    setup_wifi(); //Previously made function to connect to wifi
    connect_mqtt(); //Previously made function to connect to mqtt server

    run_classifier_init(); //Edge impulse classifier

    Serial.println("System ready"); //Print statement to make cleaner, more readable code
}

// -------------------- LOOP --------------------
void loop() { //The 
    mqtt.poll();  // keep MQTT alive

    read_audio_block(samples_for_block, BLOCK_SIZE); //Get a block of audio constantly to samples_for_block data place
    float rms = count_rms(samples_for_block, BLOCK_SIZE); //Calculate the rms of said block

    if (rms > THRESHOLD) { //If the rms is higher than the threshold we go into if bracket, if not, we reset count_value
        memcpy(&pre_buffer[pre_index * BLOCK_SIZE],  //Stores the current audio block to the prebuffer
               samples_for_block,                   //so the data wont disappear and we can save it for the full data
               BLOCK_SIZE * sizeof(int16_t));

        pre_index = (pre_index + 1) % PRE_BLOCKS; 
        count_value++; //Increase count_value by 1

        if (count_value >= PRE_BLOCKS) { //Only come inside this bracket if 4 consecutive rms threshold exceedings 
            record_audio_with_prebuffer(); //record audio with premade method
            signal_t signal; //Signal object
            signal.total_length = NUM_SAMPLES; //Total number of audio data/samples in recording, the signal needs it
            signal.get_data = microphone_audio_signal_get_data; //needed for classifier to get floating values
            ei_impulse_result_t result = {0};
            if (run_classifier_continuous(&signal, &result, false) == EI_IMPULSE_OK) { //Edge impulse classifier
                for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) { //Not completely sure, this is edge impulse classifier stuff
                    float v = result.classification[i].value; //The confidence of which value the trained model thinks the audio data is
                    const char* lbl = result.classification[i].label; //The string/character of the probable value 

                    // If value of v float is over 0.7 AND
                    if (v > 0.7f) {
                        if (strcmp(lbl, "on") == 0) { //if the character is "on" put led on
                          digitalWrite(LED_PIN, HIGH);
                        }
                        if (strcmp(lbl, "off") == 0) { //of, put the led off 
                          digitalWrite(LED_PIN, LOW);
                        }

                        // Communication with hivemq cloud broker/server. 
                        mqtt.beginMessage(mqtt_topic);
                        mqtt.print("{\"label\":\"");
                        mqtt.print(lbl);
                        mqtt.print("\",\"confidence\":");
                        mqtt.print(v, 3);
                        mqtt.print("}");
                        mqtt.endMessage();
                        //Prints in serial monitor in arduino ide
                        Serial.print("Published: ");
                        Serial.println(lbl);

                    }
                }
            }
            count_value = 0; //count_value resetted to 0
        }
    } else {
        count_value = 0; //count_value resetted to 0
    }
}
