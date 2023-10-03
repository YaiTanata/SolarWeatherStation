# SolarWeatherStation
Solar Weather Station DIY with ESP32 fully supported Home Assistant via MQTT Discovery integration

## Connection Diagram
![Connection_diagram](/docs/connection_diagram.png)

## TTGO-T18 ESP32 PIN
![TTGO_T18_PIN](/docs/ttgot18.jpg)

## Components
- TTGO T18 ESP32 Development board
- CN3791 MPPT Charging Module
- 6V 6W Solar panel
- SHT3X Temperature & Humidity sensor
- PMS3003 Paticulate matter sensor
- PMS Sensor switching circuit
  - BC337 Transistor
  - 330Ω Resistor 
- 2x 18650 Li-ion battery
- 18650 battery holder

## Platform and dependencies
The source code was written on PlatformIO IDE with Arduino framework.

Special Thank & Credit by Magi
https://www.facebook.com/magiDIY