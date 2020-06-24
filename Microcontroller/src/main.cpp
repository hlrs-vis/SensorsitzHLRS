#include <Arduino.h>
#include "HX711.h"
#include <SparkFun_ADXL345.h>

TaskHandle_t Task1, Task3;


void SensorIn( void *pvParameters );
void SendData( void *pvParameters );

class Accelerometer {
  public:
    byte id;

    long timeStamp;

    int16_t x;
    int16_t y;
    int16_t z;

    boolean print;

    Accelerometer(byte ID, bool pt){
      id = ID;
      print = pt;
    }
};

class ForceSensor {
  public:
    byte id;

    long timeStamp1;
    long force1;
    long timeStamp2;
    long force2;
    long timeStamp3;
    long force3;
    long timeStamp4;
    long force4;

    boolean print;

    ForceSensor(byte ID, bool pt){
      id = ID;
      print = pt;
    }
};



bool loopStart = false;
long startTime = 0;
ADXL345 seatAccel =ADXL345(0);
ADXL345 headAccel =ADXL345(1);
HX711 seat1;
HX711 seat2;
HX711 seat3;
HX711 seat4;
HX711 back1;
HX711 back2;
HX711 back3;
HX711 back4;

Accelerometer seatAccelObj(2, false);
Accelerometer headAccelObj(3, false);
ForceSensor seatForce(0, false);
ForceSensor backForce(1, false);

//Funktion, um alle Messdaten aus den Kraftmessdosen in ein Byte Array zu schreiben, das dann übertragen werden kann
void aggregateForce(byte pdata[], byte id, long time1, long measure1, long time2, long measure2, long time3, long measure3, long time4, long measure4) {
  pdata[0] =id;

  pdata[1] = (byte) time1;
  pdata[2] = (byte) (time1 >> 8);
  pdata[3] = (byte) (time1 >> 16);
  pdata[4] = (byte) (time1 >> 24);

  pdata[5] = (byte) measure1;
  pdata[6] = (byte) (measure1 >> 8);
  pdata[7] = (byte) (measure1 >> 16);
  pdata[8] = (byte) (measure1 >> 24);

  pdata[9] = (byte) time2;
  pdata[10] = (byte) (time2 >> 8);
  pdata[11] = (byte) (time2 >> 16);
  pdata[12] = (byte) (time2 >> 24);

  pdata[13] = (byte) measure2;
  pdata[14] = (byte) (measure2 >> 8);
  pdata[15] = (byte) (measure2 >> 16);
  pdata[16] = (byte) (measure2 >> 24);

  pdata[17] = (byte) time3;
  pdata[18] = (byte) (time3 >> 8);
  pdata[19] = (byte) (time3 >> 16);
  pdata[20] = (byte) (time3 >> 24);

  pdata[21] = (byte) measure3;
  pdata[22] = (byte) (measure3 >> 8);
  pdata[23] = (byte) (measure3 >> 16);
  pdata[24] = (byte) (measure3 >> 24);

  pdata[25] = (byte) time4;
  pdata[26] = (byte) (time4 >> 8);
  pdata[27] = (byte) (time4 >> 16);
  pdata[28] = (byte) (time4 >> 24);
  
  pdata[29] = (byte) measure4;
  pdata[30] = (byte) (measure4 >> 8);
  pdata[31] = (byte) (measure4 >> 16);
  pdata[32] = (byte) (measure4 >> 24);

}

//Funktion, um alle Daten aus Beschleunigungssensoren in ein Byte Array zu schreiben, dass dann übertragen werden kann.
void aggregateAccel(byte pdata[], byte id, long time, int x, int y, int z) {
  pdata[0] =id;

  pdata[1] = (byte) time;
  pdata[2] = (byte) (time >> 8);
  pdata[3] = (byte) (time >> 16);
  pdata[4] = (byte) (time >> 24);
  
  pdata[5] = (byte) x;
  pdata[6] = (byte) (x >> 8);

  pdata[7] = (byte) y;
  pdata[8] = (byte) (y >> 8);

  pdata[9] = (byte) z;
  pdata[10] = (byte) (z >> 8);
  
}



void SendData (void *parameter)
{
  byte bufferAc[11];
  byte bufferFc[33];

  for( ; ; ){

    if(Serial.available())
    {
      char temp = Serial.read();
      if (temp == 'k'){
       loopStart = !loopStart;
       startTime = millis();
      }
      
    }

    
    if(seatAccelObj.print && loopStart){
      aggregateAccel(bufferAc, seatAccelObj.id, seatAccelObj.timeStamp, seatAccelObj.x, seatAccelObj.y, seatAccelObj.z);
      Serial.write(bufferAc, sizeof(bufferAc));
      seatAccelObj.print = false;
    }

    if(headAccelObj.print && loopStart){
      aggregateAccel(bufferAc, headAccelObj.id, headAccelObj.timeStamp, headAccelObj.x, headAccelObj.y, headAccelObj.z);
      Serial.write(bufferAc, sizeof(bufferAc));
      headAccelObj.print = false;
    }

    if(seatForce.print && loopStart){
      aggregateForce(bufferFc, seatForce.id, seatForce.timeStamp1, seatForce.force1, seatForce.timeStamp2, seatForce.force2, seatForce.timeStamp3, seatForce.force3, seatForce.timeStamp4, seatForce.force4);
      Serial.write(bufferFc, sizeof(bufferFc));
      seatForce.print = false;
    }

    if(backForce.print && loopStart){
      aggregateForce(bufferFc, backForce.id, backForce.timeStamp1, backForce.force1, backForce.timeStamp2, backForce.force2, backForce.timeStamp3, backForce.force3, backForce.timeStamp4, backForce.force4);
      Serial.write(bufferFc, sizeof(bufferFc));
      backForce.print = false;
    }
    
    

    vTaskDelay(1);
  }

  
}

void SensorIn (void *parameter)
{
  long runTime;
  bool seatData[4];
  bool backData[4];

  for (int i = 0; i<4; ++i){
    seatData[i] = false;
    backData[i] = false;
  }

  for ( ; ; ){
    while(loopStart){

      //Zeitstempel generieren
      runTime = millis() - startTime;


      //Abfrage der Sitzsensoren
      if(seat1.is_ready() && !seatForce.print && !seatData[0]){
        seatForce.timeStamp1 = runTime;
        seatForce.force1 = (long) seat1.get_value();
        seatData[0] = true;
      }

      if(seat2.is_ready() && !seatForce.print && !seatData[1]){
        seatForce.timeStamp2 = runTime;
        seatForce.force2 = (long) seat2.get_value();
        seatData[1] = true;
      }

      if(seat3.is_ready() && !seatForce.print && !seatData[2]){
        seatForce.timeStamp3 = runTime;
        seatForce.force3 = (long) seat3.get_value();
        seatData[2] = true;
      }

      if(seat4.is_ready() && !seatForce.print && !seatData[3]){
        seatForce.timeStamp4 = runTime;
        seatForce.force4 = (long) seat4.get_value();
        seatData[3] = true;
      }
      
      //Überprüft, dass alle Sitzsensoren ausgelesen wurden und setzt print auf true
      if(seatData[0] && seatData[1] && seatData[2] && seatData[3]){
        seatForce.print = true;
        for (int i = 0; i<4; ++i){
          seatData[i] = false;
        }
      }


      //Abfrage der Lehnensensoren
      if(back1.is_ready() && !backForce.print && !backData[0]){
        backForce.timeStamp1 = runTime;
        backForce.force1 = (long) back1.get_value();
        backData[0] = true;
      }

      if(back2.is_ready() && !backForce.print && !backData[1]){
        backForce.timeStamp2 = runTime;
        backForce.force2 = (long) back2.get_value();
        backData[1] = true;
      }

      if(back3.is_ready() && !backForce.print && !backData[2]){
        backForce.timeStamp3 = runTime;
        backForce.force3 = (long) back3.get_value();
        backData[2] = true;
      }

      if(back4.is_ready() && !backForce.print && !backData[3]){
        backForce.timeStamp4 = runTime;
        backForce.force4 = (long) back4.get_value();
        backData[3] = true;
      }


      //Überprüft, dass alle Lehnensensoren ausgelesen wurden und setzt print auf true
      if(backData[0] && backData[1] && backData[2] && backData[3]){
        backForce.print = true;
        for (int i = 0; i<4; ++i){
          backData[i] = false;
        }
      }


      //Auslesen der Beschleunigungssensoren
      if (!seatAccelObj.print && runTime % 10 == 0){
        seatAccel.readAccel(&seatAccelObj.x, &seatAccelObj.y, &seatAccelObj.z);
        seatAccelObj.timeStamp = runTime;
        seatAccelObj.print = true;
      }

      if (!headAccelObj.print && runTime % 10 == 0){
        headAccel.readAccel(&headAccelObj.x, &headAccelObj.y, &headAccelObj.z);
        headAccelObj.timeStamp = runTime;
        headAccelObj.print = true;
      }

      vTaskDelay(1);

    }

    vTaskDelay(1);
  }

}

void setup() {
  // initialize serial communication at 115200Baud:
  Serial.begin(115200);

  //Set up acceleration Sensors
  seatAccel.powerOn();
  headAccel.powerOn();

  seatAccel.setRangeSetting(4);
  headAccel.setRangeSetting(4);
  seatAccel.setRate(100);
  headAccel.setRate(100);



  //Set up force Sensors
  seat1.begin(34, 14);
  seat2.begin(35, 16);
  seat3.begin(36, 17);
  seat4.begin(39, 18);
  back1.begin(5, 13);
  back2.begin(15, 25);
  back3.begin(4, 12);
  back4.begin(23, 27);

  seat1.set_scale();
  seat2.set_scale();
  seat3.set_scale();
  seat4.set_scale();
  back1.set_scale();
  back2.set_scale();
  back3.set_scale();
  back4.set_scale();

  delay(1000);

  seat1.tare();
  seat2.tare();
  seat3.tare();
  seat4.tare();
  back1.tare();
  back2.tare();
  back3.tare();
  back4.tare();

  xTaskCreatePinnedToCore(
    SensorIn
    , "Task1"
    , 1024
    , NULL
    , 1
    , NULL
    ,0
  );
  
  xTaskCreatePinnedToCore(
    SendData
    ,  "Task3"
    ,  1024  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL 
    ,  1
    );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop() {
  // put your main code here, to run repeatedly:
}





