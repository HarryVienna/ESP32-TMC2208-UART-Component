# ESP32 TMC2208 UART Component

### ESP32 library for Trinamic TMC2208 stepper motor driver

For the [Zyklochron](https://www.haraldkreuzer.net/en/news/designer-clock-3d-printer-raspberry-pi), I replaced the simple Darlington drivers with a TMC2208. Initially I ran the module in legacy mode. One motor alone ran absolutely silently, but as soon as both motors were running, a kind of whistling noise was heard. Possibly some kind of resonance. So I wanted to address the TMC2208 via the UART interface to solve the problem by fine tuning the parameters.
So I wrote this little library to be able to address the TMC2208 with an ESP32.