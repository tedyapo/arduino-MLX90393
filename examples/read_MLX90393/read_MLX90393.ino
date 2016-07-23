#include <Wire.h>
#include <Arduino.h>
#include <MLX90393.h>

MLX90393 mlx;

void setup(){
  mlx.begin();
  mlx.setGainSel(7);
  mlx.setResolution(0, 0, 0);
  Serial.begin(9600);
}

void loop(){
  MLX90393::txyz data = mlx.readField();
  Serial.print(data.x);
  Serial.print(" ");
  Serial.print(data.y);
  Serial.print(" ");
  Serial.print(data.z);
  Serial.print(" ");
  Serial.println(data.t);
  delay(1000);
}
