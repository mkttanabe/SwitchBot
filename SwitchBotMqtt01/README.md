# SwitchBotMqtt01
### SwitchBot remote controller for ESP32
Confirmed on **ESP-IDF** v3.0-dev-2561-g358c822.

### Description
ESP32 will send the "Press" command to your [SwitchBot](https://www.switch-bot.com/) via BLE, when received MQTT message '{"topic":"msg", "message":"1"}'.

Based on the following official program by Wonderlabs,Inc.

https://github.com/OpenWonderLabs/python-host/

### Demonstration

**"OK Google, Execute"**

[![demo](http://dsas.blog.klab.org/data/SwitchBot/demo.jpg)](http://dsas.blog.klab.org/data/SwitchBot/20180601_SwitchBot_ESP32_02.mp4)

### Blog

http://dsas.blog.klab.org/archives/2018-06/52295128.html (in Japanese)


Thanks for "ESP32 MQTT Library" by Mr.Turan.

https://github.com/tuanpmt/espmqtt

Thanks for CloudMQTT.

https://www.cloudmqtt.com/



Copyright 2018 KLab Inc.
