#  thermostat
esp32 acting as a floor heating thermostat
Hardware has a ds18b20 as room temperature sensor and 10k NTC as floor sensor.
NTC is used, because this is the type of original floor sensor.

Benefits of this thermostat compared to original:

1. Smaller hysteresis. Original thermostat has a mechanical relay. It cannot be switched on/off fast. This
thermostat is going to use solid state relay. This kind of relay can be switched off easily for example one
second interval.

2. Uses electricity stock price, which is got from internet, via a mqtt topic. Heating can be done only
   during cheap hours.

3. Sends its data to mqtt server, so it's behavior is easily reported.

![Alt text](images/ssd.png?raw=true "Schematic of solid state relay control")
