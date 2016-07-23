#include <Wire.h>
#include <Arduino.h>
#include <MLX90393.h>

MLX90393 mlx;

void setup(){
  uint8_t status = mlx.begin();
  Serial.begin(9600);
}

void loop(){
  MLX90393::txyz data;
  mlx.readData(data);
  Serial.print(data.x);
  Serial.print(" ");
  Serial.print(data.y);
  Serial.print(" ");
  Serial.print(data.z);
  Serial.print(" ");
  Serial.println(data.t);
  delay(1000);
}
