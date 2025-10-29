#include "esp_camera.h"
#define CAMERA_MODEL_XIAO_ESP32S3 // Has PSRAM
#include "camera_pins.h"

#include "i2c_master.h"

const uint8_t IMG_WIDTH = 240;   //have to change this var manually
const uint8_t IMG_HIGHT = 240;   //have to change this var manually
const uint8_t THRESH = 240;       //have to change this var manually
const uint8_t min_bright_rows = 5; //The min bright row to return median

camera_fb_t *fb;

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  i2c_setup();
  
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  
//-----CHANGE DEPENDING ON NEEDS----------//
//  config.frame_size = FRAMESIZE_QVGA;  // 320x240
  config.frame_size = FRAMESIZE_240X240;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
//-----CHANGE DEPENDING ON NEEDS----------//
  
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;
  

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // Setup LED FLash if LED pin is defined in camera_pins.h
  #if defined(LED_GPIO_NUM)
    setupLedFlash(LED_GPIO_NUM);
  #endif
  
//  sensor_t * s = esp_camera_sensor_get();
//  s->set_brightness(s, 1); // up the brightness just a bit
//  s->set_saturation(s, 0.6); // lower the saturation
//  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
//  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
//  s->set_wb_mode(s, 3);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
//  s->set_contrast(s, 0);       // -2 to 2
}


void loop() {
  uint64_t fr_start = esp_timer_get_time();
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }
//  uint64_t frameloadT = esp_timer_get_time() - fr_start;
//  printf("%s, %d\n", "frame loader time:", frameloadT);
   
  char x; 
  // set = p for printing image buff value printing
  if(Serial.available() > 0) {  
    x = Serial.read();
  }

//  uint64_t median_start = esp_timer_get_time();

  //Default median = 0 
  uint8_t x_median_final = 0;
  uint8_t y_median_final = 0;
  
  uint32_t pixcount = 0;
  uint16_t x_medians_real_length = 0;
  uint16_t x_medians[IMG_HIGHT];
  uint16_t y_vals_of_x_medians[IMG_HIGHT];

  for(uint16_t j = 0; j < IMG_HIGHT; j++){
    uint16_t all_x_gt_thresh [IMG_WIDTH];
    uint16_t all_x_gt_thresh_real_length = 0;

    // uint64_t innerloop = esp_timer_get_time();
    for(uint16_t i = 0; i < IMG_WIDTH; i++){

    //--------For Printing---------------//
    if(x == 'p'){
      Serial.printf("%d\n",fb->buf[pixcount]);
    }
    //--------For Printing---------------//
      
      if(fb->buf[pixcount] > THRESH){
        all_x_gt_thresh[all_x_gt_thresh_real_length] = i;
        all_x_gt_thresh_real_length += 1;
      }
      pixcount += 1;
    }
    // uint64_t innerlooptime = esp_timer_get_time() - innerloop;
    // Serial.println(innerlooptime);
    
    // MEDIAN FUNCTION (not real median, can be off by 1)
    if(all_x_gt_thresh_real_length != 0){
      // This should be already a floor division? because they are integers?
      x_medians[x_medians_real_length] = all_x_gt_thresh[all_x_gt_thresh_real_length/2];
      y_vals_of_x_medians[x_medians_real_length] = j;
      x_medians_real_length += 1;
    }
  }
  // This should be already a floor division? because they are integers?
  //-------CONDITION WHERE BASICALLY EVERYTHING IS DARK ----------------
  if(x_medians_real_length >= min_bright_rows){
    x_median_final = x_medians[x_medians_real_length/2];
    y_median_final = y_vals_of_x_medians[x_medians_real_length/2];
    }

  uint64_t send_time_start = esp_timer_get_time();  
  send_data(x_median_final,y_median_final);
//  send_data(100,100);
  uint64_t send_time = esp_timer_get_time() - send_time_start;
  printf("%s, %d\n", "sending data time:", send_time);

  

//  uint64_t medianT = esp_timer_get_time() - median_start;
//  printf("%s, %d\n", "Median algo time:", medianT);

  //--------For Printing---------------//
//  if(x == 'p'){
    Serial.printf("%d, %d\n",x_median_final, y_median_final);
//  }
  //--------For Printing---------------//

  esp_camera_fb_return(fb);

  uint64_t print_frame_time = esp_timer_get_time() - fr_start;
//  Serial.println("___________ONE PICTURE__________");
//  Serial.println(print_frame_time);
}
