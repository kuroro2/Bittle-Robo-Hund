/*!
    \file Bittle_moving.ino

    \brief Robot dog Bittle follows a line automatically

    The system made out of the robot dog Bittle in addition to the ESP32 CAM Modul
    sets the Robot in movement using a processing image software. 

    \author Ricardo Aviles, Nelly Chia, Andy Kok
*/

//! Camera libraries
#include "esp_camera.h"       ///< Header file for camera obtained from https://github.com/espressif/
#include "driver/ledc.h"      ///< To enable onboard Illumination/flash LED pin attached on 4
#include "soc/soc.h"          ///< Used to disable brownout detection
#include "soc/rtc_cntl_reg.h" ///< Used to disable brownout detection

//! Server libraries
#include <WiFi.h>
#include <WebServer.h>

//! Server variables     
const char* ssid = "Pixel kuroro";          ///<UPCA9BA752   Pixel kuroro     OPPO Reno2
const char* password = "funciona1";    ///<4z6ezfVCzrev funciona1      0138160878

WebServer server(80);

#define Pin_Wifi 12                       ///<Blue LED for succesful WiFi connection
#define LED_IDLE 13                       ///<Yellow LED for IDLE State
#define LED_MOVE 15                       ///<Green LED for active State
#define Pin_WiFi2 14                      ///<RED LED for unsuccesful WiFi connection
//IPAdress is set to http://192.168.0.116

/*!
    \brief Check for ESP32 board.
*/
#if !defined ESP32
#error Wrong board selected 
#endif
#define CAMERA_MODEL_AI_THINKER
const framesize_t FRAME_SIZE_IMAGE = FRAMESIZE_QQVGA;

//! Image Format
#define PIXFORMAT PIXFORMAT_GRAYSCALE;
#define WIDTH 160  ///< Image size Width
#define HEIGHT 120 ///< Image size Height

//! Camera exposure
/*!
    Range: (0 - 1200)
    If gain and exposure both set to zero then auto adjust is enabled
*/
int cameraImageExposure = 0;

//! Image gain
/*!
    Range: (0 - 30)
    If gain and exposure both set to zero then auto adjust is enabled
*/
int cameraImageGain = 0;
const uint8_t ledPin = 4;                  ///< onboard Illumination/flash LED pin (4)
int ledBrightness = 4;                     ///< Initial brightness (0 - 255)
const int pwmFrequency = 50000;            ///< PWM settings for ESP32
const uint8_t ledChannel = LEDC_CHANNEL_0; ///< Camera timer0
const uint8_t pwmResolution = 8;           ///< resolution (8 = from 0 to 255)
const int serialSpeed = 115200;            ///< Serial data speed to use

//! Image processing variables
int arrayCenter = WIDTH/2;
const int NUM_ROWS = 30;                   ///<number of rows of pixels that will be processed
const int NUM_COLS = WIDTH;                ///<number of columns of pixels that will be processed 
uint8_t pixels[NUM_ROWS][NUM_COLS];        ///<array filled with the pixels to be processed
uint8_t treshold_array[HEIGHT][WIDTH];     ///<array will all the pixels
int array_avrg[NUM_COLS];                  ///<1D array with only 160 pixel values averaged
int TRESHOLD;                              ///<Dynamic Treshold

//!State Machine initiators
int currentState = 0;
int lastState = -1;
bool start = false;                        ///<variable to start/stop the whole process by pressing a button 
bool messageprinted = false;               ///<variable used to limit the times the message "d" is printed

//!Timers
uint32_t lastCamera = 0;                   ///< To store time value for repeated capture
uint32_t lastServer = 0;                   ///< To store time value for repeated server request

/**************************************************************************/
/*!
  \brief  Setup function

  Initialization for:
    LEDs
    disable Brownout detection
    camera
    Server
*/
/**************************************************************************/
void setup() {
  Serial.begin(serialSpeed);                  ///< Initialize serial communication

  //!LEDs set up
  pinMode(Pin_Wifi, OUTPUT);                  ///<Status interface Pins for WiFi, and states
  pinMode(Pin_WiFi2, OUTPUT);               
  pinMode(LED_IDLE, OUTPUT);
  pinMode(LED_MOVE, OUTPUT);

  digitalWrite(LED_MOVE, LOW);
  digitalWrite(Pin_WiFi2, LOW);

  //! Server set up
  WiFi.begin(ssid, password);

  // Serial.println("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    // Serial.print(".");
  }
  // Serial.println();
  // Serial.print("Connected to ");
  // Serial.println(ssid);
  // Serial.print("IP address: ");
  // Serial.println(WiFi.localIP());

  // server.begin();
  // if (WiFi.status() != WL_CONNECTED) {
  //   digitalWrite(Pin_Wifi, LOW);
  //   digitalWrite(Pin_WiFi2, HIGH);
  // } else{
  //   digitalWrite(Pin_Wifi, HIGH);
  // }

  // Serial.print(WiFi.localIP());
  server.on("/", handleRoot);
  server.on("/start", handleStart);
  server.on("/stop", handleStop);
  server.begin();

  //! Camera Set up
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);   ///< Disable 'brownout detector'
  initialiseCamera();
  setupOnBoardFlash();
  setLedBrightness(ledBrightness);
}

/**************************************************************************/
/*!
  \brief  Loop function
  Capture image every 0.4 seconds
  Stores the pixel values in an array
  filters the array using a treshold
  Tracks the line
  Tells Bittle how to react base on the position of the line
*/
void loop() {
//!Variables used to find and track the line
int lineStart = -1;
int lineEnd = -1;
int lineCenter;
int lineWidth;
int center_damping;                        ///<15% of the total Width of the line to dampf the range to be considered as "center" of the line

  server.handleClient();

  //!Timer to communicate with ther Server to avoid unnecessary energy consumption
  // if ((unsigned long)(millis() - lastServer) <= 400ul) {
  //   server.handleClient();
  //   lastServer = millis();
  // }
  
  //!Start = false
  /*!Bittle is in IDLE State, waiting for a command. the initial command sent in this state
    is "d", which sets the Robot on stop/pause.
    Also a Yellow LED is turned on indicating the Robot is in IDLE mode*/  
  if (start == false) {    
    if (!messageprinted) {
      Serial.print("d");
      messageprinted = true;
    }
    digitalWrite(LED_IDLE, HIGH);
  } else {
    messageprinted = false;
    digitalWrite(LED_IDLE, LOW);
  }
  //!is Start = true
  /*!The main code starts running, the ESP32 takes photos every 0.4 seconds. the pixels from this picture are then processed 
     and used to find and track the line. the movements of the Robot depends on the line's position to respect to the array dimensions*/
  if (start == true) {
    digitalWrite(LED_MOVE, HIGH);
    if ((unsigned long)(millis() - lastCamera) >= 400UL) {
      lastCamera = millis(); // reset timer
      capture_still();       // function call -> capture image
      
      //!IMAGE PROCESSING
      /**************************************************************************/
      //!Calculating dynamic treshold
      for (int row = 0; row < HEIGHT; row++) {
        for (int col = 0; col < WIDTH; col++) {
          TRESHOLD += treshold_array[row][col];                         ///<adding all pixels to one variable
        }
      }
      TRESHOLD = TRESHOLD/(WIDTH * HEIGHT);                             ///<dividing this value by the amount of pixels in the image
      
      //!Taking the average of all the rows in the pixels array and saving this averages into the array_avrg array
      for (int pixel_col = 0; pixel_col < NUM_COLS; pixel_col++) {
        for (int pixel_row = 0; pixel_row < NUM_ROWS; pixel_row++) {
          array_avrg[pixel_col] += pixels[pixel_row][pixel_col];        ///<getting the sum of every column of the 30 rows array
        }
        array_avrg[pixel_col] = array_avrg[pixel_col]/(NUM_ROWS + 1);   ///<getting its average value
      }

      //!Tresholding the values in the array_avrg array for binaritation
      for (int i = 0; i < NUM_COLS; i++) {
        if (array_avrg[i] > TRESHOLD) {
          array_avrg[i] = 1;
        } else {
            array_avrg[i] = 0;
          }
      }
    
      //!Identifying the Start and End of the line
      for (int pixel = 0; pixel < NUM_COLS; pixel++) {
        if (array_avrg[pixel] == 1) {
          if (lineStart == -1) {
            lineStart = pixel;        ///<Start
          }
          lineEnd = pixel;            ///<End
        }
      }     

      //!Calculate line Center
      lineCenter = (lineStart + lineEnd)/2;
      lineWidth = lineEnd - lineStart;
      center_damping = lineWidth * 0.15;
    /**************************************************************************/

    //!Using the State Machine
    /**************************************************************************/
      //!Changing the States of the machine depending on the position of the middle point of the line respective the middle point of the Array.
      if (lineWidth > 140) {
        currentState = 4;        
      }
      if (lineCenter > arrayCenter - center_damping && lineCenter < arrayCenter + center_damping) {    
        currentState = 1;
      }
      if (lineCenter < arrayCenter - center_damping) {
        currentState = 2;
      }  
      if (lineCenter > arrayCenter + center_damping) {
        currentState = 3;
      } 
      
        

      //!Switching between States depending on the of the center pixel of the line respective to the middle pixel of the array
      /*!This if-Statement prevents the Code from sending repeated 
         commands to the Robot in this small period of time*/
      if (currentState != lastState) {                        
        StateMachine(currentState);
        lastState = currentState;
      }
    /**************************************************************************/
    }
  }
}

/**************************************************************************/
/**
  State Machine 
 */
/**************************************************************************/
void StateMachine(int currentsate) {
  switch (currentState) {        
    case 1:                     //Forward
      Serial.print("kcrF");   
      break;

    case 2:                     //Left
      Serial.print("kcrL");
      break;

    case 3:                     //Right
      Serial.print("kcrR");
      break;

    case 4:                     //180°
      Serial.print("kcrL");
      delay(30000);   
      break;
  }
}

/**************************************************************************/
/**
  Captures images
  Saves 15 Rows of the 120 rows of the original Array of pixels into a new array for further processing.
  Saves all the Pixel values into another Array to calculate the Dynamic Treshold
  \return true: successful, false: failed
 */
/**************************************************************************/
bool capture_still()
{
  camera_fb_t *frame = esp_camera_fb_get();

  if (!frame)
      return false;
  //!Array with 30 rows to use for the movement
  for (int i = 60, x = 0; i < HEIGHT - NUM_ROWS, x < NUM_ROWS; i++, x++) {
    for (int j = 0, y = 0; j < NUM_COLS, y < NUM_COLS; j++, y++) {
      pixels[x][y] = frame->buf[i * NUM_COLS + j];
    }
  }
  //!Array with all the pixel values, used to calculate the dynamic treshold
  for (int row_index = 0; row_index < HEIGHT; row_index++) {
    for (int col_index = 0; col_index < WIDTH; col_index++) {
      treshold_array[row_index][col_index] = frame -> buf[WIDTH * row_index + col_index];
    }
  }
	esp_camera_fb_return(frame);
  return true;
}

/**************************************************************************/
/**
  Changes Start value 
 */
/**************************************************************************/
void handleStart() {
  start = true;
  server.send(200, "text/plain", "Robot started");
}

void handleStop() {
  start = false;
  server.send(200, "text/plain", "Robot stopped");
}
void handleRoot() {
  String message = "ESP32-CAM is online!";
  server.send(200, "text/html", message);
}










/**************************************************************************/
//! Camera setting
/*!
    Camera settings for CAMERA_MODEL_AI_THINKER OV2640
    Based on CameraWebServer sample code by ESP32 Arduino

*/
/**************************************************************************/
#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM 32
  #define RESET_GPIO_NUM -1
  #define XCLK_GPIO_NUM 0
  #define SIOD_GPIO_NUM 26
  #define SIOC_GPIO_NUM 27
  #define Y9_GPIO_NUM 35
  #define Y8_GPIO_NUM 34
  #define Y7_GPIO_NUM 39
  #define Y6_GPIO_NUM 36
  #define Y5_GPIO_NUM 21
  #define Y4_GPIO_NUM 19
  #define Y3_GPIO_NUM 18
  #define Y2_GPIO_NUM 5
  #define VSYNC_GPIO_NUM 25
  #define HREF_GPIO_NUM 23
  #define PCLK_GPIO_NUM 22
#endif

/**************************************************************************/
/**
  Initialise Camera
  Set camera parameters
  Based on CameraWebServer sample code by ESP32 Arduino
  \return true: successful, false: failed
 */
/**************************************************************************/
bool initialiseCamera()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT;
  config.frame_size = FRAME_SIZE_IMAGE;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  // Check the esp32cam board has a PSRAM chip installed (extra memory used for storing captured images)
  // Note: if not using "AI thinker esp32 cam" in the Arduino IDE, PSRAM must be enabled
  if (!psramFound())
  {
    digitalWrite(Pin_WiFi2, HIGH);
    config.frame_size = FRAMESIZE_CIF;
  }

  esp_err_t camerr = esp_camera_init(&config); // initialise the camera

  if (camerr != ESP_OK)
  {
    digitalWrite(Pin_WiFi2, HIGH);
  }

  cameraImageSettings(); // Apply custom camera settings

  return (camerr == ESP_OK); // Return boolean result of camera initialisation
}

/**************************************************************************/
/**
  Camera Image Settings
  Set Image parameters
  Based on CameraWebServer sample code by ESP32 Arduino
  \return true: successful, false: failed
 */
/**************************************************************************/
bool cameraImageSettings()
{
  sensor_t *s = esp_camera_sensor_get();
  if (s == NULL)
  {
    digitalWrite(Pin_WiFi2, HIGH);
    return 0;
  }
  // if both set to zero enable auto adjust
  if (cameraImageExposure == 0 && cameraImageGain == 0)
  {
    // enable auto adjust
    s->set_gain_ctrl(s, 1);     // auto gain on
    s->set_exposure_ctrl(s, 1); // auto exposure on
    s->set_awb_gain(s, 1);      // Auto White Balance enable (0 or 1)
  }
  else
  {
    // Apply manual settings
    s->set_gain_ctrl(s, 0);                   // auto gain off
    s->set_awb_gain(s, 1);                    // Auto White Balance enable (0 or 1)
    s->set_exposure_ctrl(s, 0);               // auto exposure off
    s->set_agc_gain(s, cameraImageGain);      // set gain manually (0 - 30)
    s->set_aec_value(s, cameraImageExposure); // set exposure manually  (0-1200)
  }
  return true;
}

/**************************************************************************/
/**
  Setup On Board Flash
  Initialize on board LED with pwm channel
 */
/**************************************************************************/
void setupOnBoardFlash()
{
  ledcSetup(ledChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(ledPin, ledChannel);
}

/**************************************************************************/
/**
  Set Led Brightness
  Set pwm value to change brightness of LED
 */
/**************************************************************************/
void setLedBrightness(byte ledBrightness)
{
  ledcWrite(ledChannel, ledBrightness);
}