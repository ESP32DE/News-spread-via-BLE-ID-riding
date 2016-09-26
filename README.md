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
The array size is limited to be 31 bytes(octets).

char *adv_name = "ESP-BLE-HELLO";
    uint8_t name_len = (uint8_t)strlen(adv_name);
 >> uint8_t adv_data[31] = {0x02, 0x01, 0x06, 0x0, 0x09};  <<
    uint8_t adv_data_len;
    
    adv_data[3] = name_len + 1;
    for (int i=0; i<name_len; i++) {
        adv_data[5+i] = (uint8_t)adv_name[i];
}

This is a limit imposedon the scan response data by the BLE spec, 
and thus cannot be raised for any device that is spec compliant

As android docs say, "Valid Bluetooth names are a maximum of 248 bytes using UTF-8 encoding, 
although many remote devices can only display the first 40 characters, 
and some may be limited to just 20."

Try it!
i try with 26 bytes ( 31 - 26 = 5** )
	adv_data[5+i] = (uint8_t)adv_name[i];
	

