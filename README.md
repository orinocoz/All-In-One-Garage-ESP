# All-In-One-Garage-ESP
All-In-One Garage Controller on ESP8266 (WeMos D1 Mini)

Communicates over WiFi to MQTT broker, for Home Assistant or other system.
As built, for two car bays with one door.

## Parts List

- [WeMos D1 Mini Module](<https://www.aliexpress.com/item/D1-mini-Mini-NodeMcu-4M-bytes-Lua-WIFI-Internet-of-Things-development-board-based-ESP8266-by/32644199530.html>)
- [Power Supply 5V Micro USB](<https://www.amazon.com/Raspberry-Power-Supply-Adapter-Charger/dp/B0719SX3GC/ref=sr_1_2_sspa/147-0126656-0783643?ie=UTF8&qid=1535657227&sr=8-2-spons&keywords=raspberry+pi+power+supply&psc=1>)
- [5V Relay Module (2 needed, or a 2 channel relay, if for two doors)](<https://www.aliexpress.com/item/one-1-channel-relay-module-with-optocoupler-isolation-fully-compatible-with-3-3V-and-5V-Signal/32869288345.html?spm=a2g0s.12269583.0.0.a0b91edfH6f3go>) - Relay needs to trip with 3.3V and **_low current_**.
- [Magnetic Contact Switch (2 needed if for two doors)](<https://www.aliexpress.com/item/NO-NC-Wired-Door-Magnetic-Contact-Switch-alarm-Reed-switches-door-sensor-for-Home-Alarm-System/32832532279.html>)
- Jumper wires, etc.

Following are optional:
- [HC-SR04 Distance Sensor (1/5, 2 needed if for two car bays)](<https://www.amazon.com/gp/product/B01GVDV6DY/ref=oh_aui_detailpage_o01_s02?ie=UTF8&psc=1>)
- [Passive Buzzer Alarm Module](<https://www.dx.com/p/passive-low-level-buzzer-alarm-module-for-arduino-dc-3-5v-5v-2017987#.XAF3AeJlCUk>)
- [RJ11 Keystone Jack (2 needed if for one car bay, 4 if for two car bays, 6 if for two doors/two bays)](<https://www.homedepot.com/p/Commercial-Electric-Voice-Grade-Jack-White-5023-WH/206427969>)
- [DHT22 AM2302 Digital Temperature and Humidity Sensor](<https://www.amazon.com/TOOGOO-Digital-Temperature-Humidity-Raspberry/dp/B0757F4KK3/ref=pd_day0_hl_328_8?_encoding=UTF8&pd_rd_i=B0757F4KK3&pd_rd_r=e7c647f0-ad45-11e8-bcdb-932140fb8169&pd_rd_w=BsSsw&pd_rd_wg=0gMnm&pf_rd_i=desktop-dp-sims&pf_rd_m=ATVPDKIKX0DER&pf_rd_p=ad07871c-e646-4161-82c7-5ed0d4c85b07&pf_rd_r=XEK41RAJAW9FSF05109K&pf_rd_s=desktop-dp-sims&pf_rd_t=40701&psc=1&refRID=XEK41RAJAW9FSF05109K>)
- [HC-SR501 PIR (motion) sensor](<https://www.adafruit.com/product/189>)
- [Mini Breadboard Kit (1/6)](<https://www.amazon.com/gp/product/B01EV6SBXQ/ref=oh_aui_detailpage_o05_s00?ie=UTF8&psc=1>)
- Main Project Case ([Garage Parts - Main - Bottom.stl](<Garage Parts - Main - Bottom.stl>) and [Garage Parts - Main - Top.stl](<Garage Parts - Main - Top.stl>))
- Remote Project Case if for two car bays ([Garage Parts - Remote - Bottom.stl](<Garage Parts - Remote - Bottom.stl>) and [Garage Parts - Remote - Top.stl](<Garage Parts - Remote - Top.stl>))

Approximate total cost $50.00

## Wiring for one door and two cars

- D1 - Speaker I/O
- D2 - DHT22 Out
- D3 - Don't use, controls run/flash mode
- D4 - Controls LED, but could be used
- G - Left Echo Gnd - Right Echo Gnd (to jack) - DHT22 Gnd(-) - Motion Gnd - Left Relay Gnd - Speaker Gnd - Left Mag Switch N/C
- 5v - Left Echo Vcc - Right Echo Vcc (to jack) - DHT22 Vcc(+) - Motion +5v - Left Relay 5v - Speaker Vcc

- D0 - Right Echo Trig/Echo (to jack)
- D5 - Left Echo Trig/Echo
- D6 - Motion Out
- D7 - Left Relay
- D8 - Left Mag Switch Comm
- 3v - (Maybe Speaker Vcc)

Note: Building for two doors and two bays with all the options will require more ports than the WeMos D1 Mini Module normally supplies.  (The TX/RX ports could be used if you don't need the serial monitor for debugging, but this would require some reprogramming of the sketch.)
