# Intervalometer_Hardware
Build an Intervalometer use minimal Hardware. This can be used for every Camera which can be triggered with a remote release.

### Features so far:
 * Connect Bluetooth module to mobile device and transfere static function commands (once).
 * Running function commands (trigger the shutter in defined intervals) after transmission.
 * Pause execution via switch
 * LED indicator for current state. 
 
## Todos
 - [ ] Housing.
 - [ ] Support mirror lockup
 - [x] Go into low power mode between shutter trigger
 - [ ] Decide on power supply (BT Module needs 5.1V, Attiny beween 2.7–5.5V)
 - [ ] Disable Bluetooth programmatically (or manual with a switch?)
 - [ ] battery indicator
 - [ ] manual how to build/flash
 - [ ] Clock-callibration Mode (uncallibrated there can be a relative error each second of 0.06s - 0.018s (from measurements))


