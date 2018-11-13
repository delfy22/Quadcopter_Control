Code to read and write data using iBus protocol over one serial channel on Arduino. As iBus is one wire bidirectional, this could rx and tx to one iBus connection by placing a resistor between the rx and tx pins. Or, it could rx from one iBus connection and tx to another.

Note: On the Arduino Mega, Serial is TX0 and RX0 but is used by the USB serial connection too so will interfere with programming and cannot use the serial
monitor. Instead, use Serial1, 2, or 3.

iBus reading adapted from: https://gitlab.com/timwilkinson/FlySkyIBus
iBus writing adapted from: https://github-mirror.open.netease.com/wdcossey/ppm-to-ibus-serial
