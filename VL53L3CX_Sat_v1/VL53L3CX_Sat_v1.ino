/**
 ******************************************************************************
 * @file    VL53L3CX_Sat_HelloWorld.ino
 * @author  SRA
 * @version V1.0.0
 * @date    30 July 2020
 * @brief   Arduino test application for the STMicrolectronics VL53L3CX
 *          proximity sensor satellite based on FlightSense.
 *          This application makes use of C++ classes obtained from the C
 *          components' drivers.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2020 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*
 * To use this sketch you need to connect the VL53L3CX satellite sensor directly to the Nucleo board with wires in this way:
 * pin 1 (Interrupt) of the VL53L3CX satellite connected to pin A2 of the Nucleo board 
 * pin 2 (SCL_I) of the VL53L3CX satellite connected to pin D15 (SCL) of the Nucleo board with a Pull-Up resistor of 4.7 KOhm
 * pin 3 (XSDN_I) of the VL53L3CX satellite connected to pin A1 of the Nucleo board
 * pin 4 (SDA_I) of the VL53L3CX satellite connected to pin D14 (SDA) of the Nucleo board with a Pull-Up resistor of 4.7 KOhm
 * pin 5 (VDD) of the VL53L3CX satellite connected to 3V3 pin of the Nucleo board
 * pin 6 (GND) of the VL53L3CX satellite connected to GND of the Nucleo board
 * pins 7, 8, 9 and 10 are not connected.
 */
/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
#include <Wire.h>
#include <vl53lx_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#include <ArduinoJson.h>

#define DEV_I2C Wire
#define SerialPort Serial

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif
#define LedPin LED_BUILTIN

// Components.
VL53LX sensor_vl53lx_sat(&DEV_I2C, A1);

// An example demonstrating how to control the Adafruit Dot Star RGB LED
// included on board the ItsyBitsy M4 board.

#include <Adafruit_DotStar.h>

// There is only one pixel on the board
#define NUMPIXELS 1 

//Use these pin definitions for the ItsyBitsy M4
#define DATAPIN    41
#define CLOCKPIN   40

Adafruit_DotStar strip(NUMPIXELS, DATAPIN, CLOCKPIN, DOTSTAR_BRG);

//RGB col define
uint32_t startup = strip.Color(24, 217, 214);
uint32_t outTol = strip.Color(255, 0, 0);
uint32_t twoObj = strip.Color(255, 204, 255);
  
uint32_t full = strip.Color(255, 196, 0);
uint32_t half = strip.Color(208, 255, 0);
uint32_t empty = strip.Color(0, 255, 0);
uint32_t hold = strip.Color(0, 255, 247);

int cycle;
float distance;
float upperTol = 88;//distence to bottom of bin when full 
float lowerTol = 5;//distence to bottom of bin when empty 


/* Setup ---------------------------------------------------------------------*/

void setup()
{
  //intergrated RGB led
  strip.begin(); // Initialize pins for output
  strip.setBrightness(80);
  strip.show();  // push col 
  
   // Led.
   pinMode(LedPin, OUTPUT);

   // Initialize serial for output.
   SerialPort.begin(115200);
   SerialPort.println("Starting...");

   // Initialize I2C bus.
   DEV_I2C.begin();

   // Configure VL53LX satellite component.
   sensor_vl53lx_sat.begin();

   // Switch off VL53LX satellite component.
   sensor_vl53lx_sat.VL53LX_Off();

   //Initialize VL53LX satellite component.
   sensor_vl53lx_sat.InitSensor(0x12);

   // Start Measurements
   sensor_vl53lx_sat.VL53LX_StartMeasurement();
}

void loop()
{
   VL53LX_MultiRangingData_t MultiRangingData;
   VL53LX_MultiRangingData_t *pMultiRangingData = &MultiRangingData;
   uint8_t NewDataReady = 0;
   int no_of_object_found = 0, j;
   int status;
   String report;
   int ans;
   StaticJsonDocument<16> dataOut;

   do
   {
      status = sensor_vl53lx_sat.VL53LX_GetMeasurementDataReady(&NewDataReady);
   } while (!NewDataReady);

   //Led on
   digitalWrite(LedPin, HIGH);

   if((!status)&&(NewDataReady!=0))
   {
      status = sensor_vl53lx_sat.VL53LX_GetMultiRangingData(pMultiRangingData);
      no_of_object_found=pMultiRangingData->NumberOfObjectsFound;

      //make into avrage 
      if (cycle > 4){
        distance /= 4;

        if ((distance > upperTol) || (distance < lowerTol)){
          set_col(outTol);
        }
        if ((distance < upperTol) && (distance > lowerTol)){
          
          ans = ((distance - upperTol) / (lowerTol - upperTol)) * 100;
          
          //set relivent rgb colour
          if (ans =< 80){
            set_col(full);
          } else if (ans =< 20){
            set_col(half);
          } else if (ans =< 0){
            set_col(empty);
          }
          
          //convert to Json and dump to serial can be interprited by python right now
          
          dataOut["percentage"] = ans;

          serializeJson(dataOut, Serial);
          
          // 5min hold cycle
          //set_col(hold);
          //delay(300);
        
          
        }

        cycle = 0;
        distance = 0;
      }
      //for(j=0;j<no_of_object_found;j++)
      //{
        //SerialPort.print(int(no_of_object_found));
        //SerialPort.print("\t");
         if(int(no_of_object_found)=1){
           distance += pMultiRangingData->RangeData[0].RangeMilliMeter/10;
           cycle += 1;
           delay(1);
         }
          if(int(no_of_object_found)>=2){
            set_col(twoObj);
            delay(1);
          }
      //}
      
      if (status==0)
      {
         status = sensor_vl53lx_sat.VL53LX_ClearInterruptAndStartMeasurement();
      }
   }

   digitalWrite(LedPin, LOW);
}

void set_col(uint32_t col)
{
  strip.setPixelColor(0, col);
  strip.show();  // push col 
}
