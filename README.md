# iperf client for Heltec ESP32 Wifi Kit

This is an implementation of the iperf network speed measurement client protocol (TCP)
for a Heltec Wifi Kit 32 or LoRa Kit, written for the arduino IDE

## Getting Started

If necessary install the Heltec Library for arduino according to 
https://docs.heltec.cn/#/en/user_manual/how_to_install_esp32_Arduino
After the download of the program in the esp32, it starts to connect to your wifi. 
After successful connection it opens a TCP iperf connection to the specified server. 
The results are displayed on the OLED. After 5 seconds it stops. 
A new measuremnt can be startet by pressing the prg/boot button. The maximum bandwidth I saw was 11 MBit/s.

### Prerequisites

Just open the sketch file in arduino an check the settings in tools/werkzeuge.
You should edit the (hard codes) SSID and password in the source file before using.
The iperf server ip address has also to be adjusted.


### Installing

not necessary


## Built With

* arduino v1.8.9
* Heltec library for arduino


## Authors

* Agrippus [agrippus](https://github.com/agrippus)

## License

This project is licensed under the MIT License

## Acknowledgments

* Heltec/Arduino

