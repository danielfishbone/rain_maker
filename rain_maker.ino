#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <ESP32Encoder.h>
#define CLK 4 // CLK ENCODER 
#define DT 15 // DT ENCODER 
 
#define DEFAULT_POWER_MODE true
#define DEFAULT_DIMMER_LEVEL 50
#define MIN_BRIGHTNESS 0
#define MAX_BRIGHTNESS 255
const char *service_name = "PROV_1234";
const char *pop = "abcd1234";


//GPIO for virtual device
static int gpio_0 = 5;
static int gpio_dimmer = 16;


bool switch_state = false;
int brightness = DEFAULT_DIMMER_LEVEL;
long newBrightness = DEFAULT_DIMMER_LEVEL;

byte brightnessTolerance = 5;

//***********************************parameters for the pwm for LED
const int freq = 1000;        //setting frequency
const int resolution = 8;    // using 8 bit resolution (this has a max value of 255)
const int ledChannel = 0; 

const int ledPin = 18;      //  r  the output pin
const int timeout = 200;    //  a little delay after you turn the potentiometer 
unsigned long timestamp =0;
boolean changeFlag = false;
 ESP32Encoder encoder;
// The framework provides some standard device types like switch, lightbulb, fan, temperature sensor.
// But, you can also define custom devices using the 'Device' base class object, as shown here
static LightBulb my_device("Bulb", &gpio_dimmer);

void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id) {
        case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32S2
            Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
            printQR(service_name, pop, "softap");
#else
            Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
            printQR(service_name, pop, "ble");
#endif
            break;
        case ARDUINO_EVENT_PROV_INIT:
            wifi_prov_mgr_disable_auto_stop(10000);
            break;
        case ARDUINO_EVENT_PROV_CRED_SUCCESS:
            wifi_prov_mgr_stop_provisioning();
            break;
        default:;
    }
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
    const char *device_name = device->getDeviceName();
    const char *param_name = param->getParamName();

    if(strcmp(param_name, "Power") == 0) {
        Serial.printf("Received value = %s for %s - %s\n", val.val.b? "true" : "false", device_name, param_name);
        switch_state = val.val.b;
        //(switch_state == false) ? digitalWrite(gpio_dimmer, LOW) : digitalWrite(gpio_dimmer, HIGH);
        param->updateAndReport(val);
    } else if (strcmp(param_name, "Level") == 0) {
        Serial.printf("\nReceived value = %d for %s - %s\n", val.val.i, device_name, param_name);
        newBrightness = val.val.i;
        encoder.setCount(newBrightness);
        Serial.printf("new brightness : %d", newBrightness);
        updateSteps();
        param->updateAndReport(val);
    }
}

void setup()
{
    Serial.begin(115200); //bedin Serial communication
    encoder.attachHalfQuad ( DT, CLK ); //initialize encoder object
    encoder.setCount ( DEFAULT_DIMMER_LEVEL ); // set initial brightness level of the LED 
    pinMode(gpio_0, INPUT); 
    pinMode(gpio_dimmer, OUTPUT);
    pinMode(ledPin, OUTPUT);
    digitalWrite(gpio_dimmer, DEFAULT_POWER_MODE);
    ledcSetup(ledChannel, freq, resolution);
    ledcAttachPin(ledPin, ledChannel);
    Node my_node;
    my_node = RMaker.initNode("ESP RainMaker Node");

    //Create custom dimmer device
    my_device.addNameParam();
    my_device.addPowerParam(DEFAULT_POWER_MODE);
    my_device.assignPrimaryParam(my_device.getParamByName(ESP_RMAKER_DEF_POWER_NAME));

    //Create and add a custom level parameter
    Param level_param("Level", "custom.param.level", value(DEFAULT_DIMMER_LEVEL), PROP_FLAG_READ | PROP_FLAG_WRITE);
    level_param.addBounds(value(MIN_BRIGHTNESS ), value(MAX_BRIGHTNESS ), value(1));
    level_param.addUIType(ESP_RMAKER_UI_SLIDER);
    my_device.addParam(level_param);

    my_device.addCb(write_callback);

    //Add custom LightBulb device to the node
    my_node.addDevice(my_device);

    //This is optional
    RMaker.enableOTA(OTA_USING_TOPICS);
    RMaker.enableTZService();

    RMaker.enableSchedule();

    RMaker.enableScenes();

    RMaker.start();

    WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32S2
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#else
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#endif
}

void loop(){
   newBrightness = encoder.getCount();
   updateSteps();

    if(digitalRead(gpio_0) == LOW) { // if Push button pressed

        // Key debounce handling
        delay(100);
        int startTime = millis();
        while(digitalRead(gpio_0) == LOW) delay(50);
        int endTime = millis();

        if ((endTime - startTime) > 10000) {
          // If key pressed for more than 10secs, reset all
          Serial.printf("Reset to factory.\n");
          RMakerFactoryReset(2);
        } else if ((endTime - startTime) > 3000) {
          Serial.printf("Reset Wi-Fi.\n");
          // If key pressed for more than 3secs, but less than 10, reset Wi-Fi
          RMakerWiFiReset(2);
        }
      else {
          // Toggle device state
          switch_state = !switch_state;
          update_stat();
      }
    }
   //******************************************************
   // switch logic comes here 
  // here, the switch state reflects the on or off status of the light bulb device (or on,off in the app)
  // brightness reflects the the  level (this changes with rotation of the the encoder or slider in the app)

   if(switch_state){ // if the switch is on (in the app or if the encoder button is toggled)
    ledcWrite(ledChannel,brightness); 
    }

    else{
      ledcWrite(ledChannel,0);
      }
    if (newBrightness != brightness){ // when the brightness value gotten from the app is not equal to 
                                      // the current brightness value 
      changeFlag = true;              // set change flag to true (this'll be used later to synchronize the parameters)
      brightness = newBrightness;
      timestamp = millis();
      updateSteps();// this updates the encoder steps 
      }
    if (millis() - timestamp >= timeout && changeFlag ==true) {
      update_stat();// this function synchronizes device status with app 
      
      // this flag helps to keep track of when parameters change , 
      //so it'll only update when parameters change
      changeFlag = false; 
      } 

    //************************************************************  
     delay(100);
   
}
void update_stat(){// this function synchronizes device status with app 
          my_device.updateAndReportParam("Level", brightness); // update adpp with current brightness level
          Serial.printf("Toggle State to %s.\n", switch_state ? "true" : "false");
          Serial.printf("Brightness State to %d.\n", brightness);
          my_device.updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state);// update adpp with current switch
  }
void updateSteps(){ // this updates the encoder steps and keep it within boundaries 
  if(newBrightness >= MAX_BRIGHTNESS ){
    encoder.setCount(MAX_BRIGHTNESS );
    newBrightness = MAX_BRIGHTNESS ;
  }
  else if(newBrightness <= MIN_BRIGHTNESS ){
    encoder.setCount(MIN_BRIGHTNESS );
    newBrightness = MIN_BRIGHTNESS ;
    }
  }  
