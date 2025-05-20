#include <SPI.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <stdlib.h> 

//#include "esp_heap_caps.h"

extern "C" {
  #include "tommath.h"
}


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

TFT_eSPI tft = TFT_eSPI();

#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240

int MAX_ITER = 100;

#define SCALE 10000 
uint8_t rgb565Buffer[SCREEN_WIDTH * SCREEN_HEIGHT];
uint16_t palette[256];

unsigned long start,stop;

TaskHandle_t taskCore0;
TaskHandle_t taskCore1;

float minReal,maxReal,minImag, maxImag;

uint8_t imageIndices[SCREEN_WIDTH * SCREEN_HEIGHT];

SemaphoreHandle_t arrayMutex;

int mapFixed(int pos, int size, float min, float max) {
  float coord = min + (max - min) * pos / size;
  return (int)(coord * SCALE);
}


void renderMandelbrot() {
  mp_int x0, y0, x, y, xtemp, xx, yy, sum, two, four;
  mp_init_multi(&x0, &y0, &x, &y, &xtemp, &xx, &yy, &sum, &two, &four, NULL);
  //int SCALE = 1000;

  mp_set_u64(&two, 2);
  mp_set_u64(&four, 4 * SCALE);  // 4.0 scaled

  for (int py = 0; py < SCREEN_HEIGHT; py++) {
    int y0_fixed = mapFixed(py, SCREEN_HEIGHT, -1.5, 1.5);

    for (int px = 0; px < SCREEN_WIDTH; px++) {
      int x0_fixed = mapFixed(px, SCREEN_WIDTH, -2.0, 1.0);
      // Map screen (px, py) to complex plane
      mp_set_i32(&x0, x0_fixed);
      mp_set_i32(&y0, y0_fixed);
      //mp_set_u64(&x0, ((int64_t)(px) * 3000 / SCREEN_WIDTH) - 2000);   // -2000 to 1000
      //mp_set_u64(&y0, ((int64_t)(py) * 3000 / SCREEN_HEIGHT) - 1500);  // -1500 to 1500

      mp_zero(&x);
      mp_zero(&y);

      int iter = 0;
      while (iter < MAX_ITER) {
        // xx = x*x / SCALE
        mp_mul(&x, &x, &xx);
        mp_div_d(&xx, SCALE, &xx, NULL);

        // yy = y*y / SCALE
        mp_mul(&y, &y, &yy);
        mp_div_d(&yy, SCALE, &yy, NULL);

        // sum = xx + yy
        mp_add(&xx, &yy, &sum);

        // if (sum > 4.0) break
        if (mp_cmp(&sum, &four) == MP_GT)
          break;

        // xtemp = x*x - y*y + x0
        mp_sub(&xx, &yy, &xtemp);
        mp_add(&xtemp, &x0, &xtemp);

        // y = 2*x*y / SCALE + y0
        mp_mul(&x, &y, &y);
        mp_mul(&y, &two, &y);
        mp_div_d(&y, SCALE, &y, NULL);
        mp_add(&y, &y0, &y);

        mp_copy(&xtemp, &x);

        iter++;
      }

      // Choose color based on iteration count
      uint16_t color = (iter == MAX_ITER)
                       ? TFT_BLACK
                       : tft.color565(iter * 3, iter * 2, iter * 5);
      //tft.drawPixel(px, py, color);
      rgb565Buffer[(py*SCREEN_WIDTH)+px] = color;
    }
  }

  mp_clear_multi(&x0, &y0, &x, &y, &xtemp, &xx, &yy, &sum, &two, &four, NULL);
  tft.pushImage(0,0,320,240,rgb565Buffer,palette);
}



void drawMandelbrot(float minReal, float maxReal, float minImag, float maxImag, int itera) {
  for (int py = 0; py < SCREEN_HEIGHT/2; py++) {
    for (int px = 0; px < SCREEN_WIDTH; px++) {
      float x0 = minReal + (maxReal - minReal) * px / SCREEN_WIDTH;
      float y0 = minImag + (maxImag - minImag) * py / (SCREEN_HEIGHT/2);

      float x = 0.0;
      float y = 0.0;
      int iteration = 0;

      while (x * x + y * y <= 4.0 && iteration < itera) {
        float xtemp = x * x - y * y + x0;
        y = 2 * x * y + y0;
        x = xtemp;
        iteration++;
      }
      uint16_t color = (iteration == itera) ? TFT_BLACK : tft.color565(iteration * 2, iteration * 5, iteration * 3);
      if (xSemaphoreTake(arrayMutex, portMAX_DELAY) == pdTRUE) {
              rgb565Buffer[(py*SCREEN_WIDTH)+px] = color;
              xSemaphoreGive(arrayMutex);  
        }
      
    }
  }
}

void drawMandelbrot2(float minReal, float maxReal, float minImag, float maxImag, int itera) {
  for (int py = 0; py < SCREEN_HEIGHT/2; py++) {
    for (int px = 0; px < SCREEN_WIDTH; px++) {
      float x0 = minReal + (maxReal - minReal) * px / SCREEN_WIDTH;
      float y0 = minImag + (maxImag - minImag) * py / (SCREEN_HEIGHT/2);

      float x = 0.0;
      float y = 0.0;
      int iteration = 0; 

      while (x * x + y * y <= 4.0 && iteration < itera) {
        float xtemp = x * x - y * y + x0;
        y = 2 * x * y + y0;
        x = xtemp;
        iteration++;
      }

      uint16_t color = (iteration == itera) ? TFT_BLACK : tft.color565(iteration * 2, iteration * 5, iteration * 3);

      if (xSemaphoreTake(arrayMutex, portMAX_DELAY) == pdTRUE) {
              rgb565Buffer[((py+(SCREEN_HEIGHT/2))*SCREEN_WIDTH)+px] = color;
              xSemaphoreGive(arrayMutex); 

        }
    }
  }
}


void Task0(void *pvParameters) {
  drawMandelbrot(minReal,maxReal,minImag,minImag+((maxImag-minImag)/2),MAX_ITER);
  vTaskDelete(NULL); 
}

void Task1(void *pvParameters) {
  drawMandelbrot2(minReal,maxReal,minImag+((maxImag-minImag)/2),maxImag,MAX_ITER);
  vTaskDelete(NULL);  // Kill this task
}

void setup() {
  Serial.begin(115200);
 

  tft.init();
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);

  for (int i = 0; i < 256; i++) {
    palette[i] = tft.color565(i, 255 - i, i / 2);
  }

  //renderMandelbrot();
  //while(true){};
  /*
  minReal = -2.0;
  maxReal = 1.0;
  minImag = -1.5;
  maxImag = 1.5;
  */

  minReal = -0.573;
  maxReal = -0.5527;
  minImag = 0.6316;
  maxImag = 0.6519;
  MAX_ITER = 500;

/*
  minReal = -0.7478;
  maxReal = -0.7465;
  minImag = 0.0996;
  maxImag = 0.1009;
  MAX_ITER = 2500;
  */
   arrayMutex = xSemaphoreCreateMutex();




    start = millis();
    xTaskCreatePinnedToCore(Task0, "Core0", 10000, NULL, 1, &taskCore0, 0);
    xTaskCreatePinnedToCore(Task1, "Core1", 10000, NULL, 1, &taskCore1, 1);
}


void rotatePaletteRight(uint16_t* palette, int size) {
    uint16_t last = palette[size - 1];
    for (int i = size - 1; i > 0; --i)
        palette[i] = palette[i - 1];
    palette[0] = last;
}

void mapToRGB565(const uint8_t* indexBuffer, const uint16_t* palette, uint16_t* out, int size) {
    for (int i = 0; i < size; ++i)
        out[i] = palette[indexBuffer[i]];
}

void loop() {
  static bool done = false;
  int s = 1;
  if (!done && eTaskGetState(taskCore0) == eDeleted && eTaskGetState(taskCore1) == eDeleted) {
    tft.pushImage(0,0,320,240,rgb565Buffer,palette);
    done = true;
    stop = millis();  
    }
  Serial.print("Tempo ");
  Serial.println(stop-start); 
  delay(3000);
  while (true){
      for (int i = 0; i < 76800; i++) {
        rgb565Buffer[i] = (rgb565Buffer[i]+1)%256;
      }
      tft.pushImage(0,0,320,240,rgb565Buffer,palette);
      delay(200);
    }
  }
