# Lite Storm

Codebase for visible light communication prototype.

TIVA TM4C1294XL microcontroller recieves data over serial UART and transmit via manchester encoding to an LED.

The data is recieved by a photodiode which is then transmitted back over Serial UART through the reciever microcontroller (also TIVA-C)

Runs at up to 120 kbps (and probably faster).
