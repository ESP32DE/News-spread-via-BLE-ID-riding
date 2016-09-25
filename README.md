# News-spread-via-BLE-ID-riding
fun factor 

# ESP32 Wi-Fi + Bluetooth Combo Chip
http://espressif.com/en/products/hardware/esp32/overview

# Espressif IoT Development Framework with the ESP32
https://github.com/espressif/esp-idf

# ESP-IDF ble_advertising app 
https://github.com/espressif/esp-idf/tree/master/examples/04_ble_adv


Example fun doing mod on the ESP-IDF- ble-advertising app

The src let you spread-via-BLE-ID-riding BLE Identifier messages,
that you can set in your code as routine, or set by press a button, 
or set by incomming UART message as instruction code for the ble.

what was done:
simple changed the setup routine and reset in each step the Identifier
by an User message.

what you can do:

- add your button routine and setup example the button with a new Identifier.
  you can then setup the BLE Identifier by button.

- add your UART incomming instruction message and switch by incomming char 
  your case for the wanted Identifier.
  
- build an ESP32 scanner for the Identifier and evalute the Identifier

- ... your idea ...


have phun!

best wishes
rudi ;-)


short video:
https://www.youtube.com/watch?v=Y9myTrn75ig

forum task:
http://esp32.com/viewtopic.php?f=18&t=257

twitter:
https://twitter.com/eMbeddedHome/status/780160242948726784

contact:
rudi@eMbeddedHome.de


FYI: 
The BLE spec says that the device name field may be between 0 and 248 octets.
Try it!
;-)
