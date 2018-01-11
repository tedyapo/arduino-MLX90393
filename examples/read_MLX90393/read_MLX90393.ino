#include <Wire.h>
#include <Arduino.h>
#include <MLX90393.h>

// prints Bx By Bz (in uT) and temperature (C) to serial console

MLX90393 mlx;

void setup(){
  // DRDY line connected to A3 (omit third parameter to used timed reads)
  uint8_t status = mlx.begin(0, 0);
  Serial.begin(9600);
}

void loop(){
  MLX90393::txyz data;
  if (Serial.available()){
    Serial.read();
    mlx.readData(data);
    Serial.print(data.x);
    Serial.print(" ");
    Serial.print(data.y);
    Serial.print(" ");
    Serial.print(data.z);
    Serial.print(" ");
    Serial.println(data.t);
  //delay(1);
  }
}
