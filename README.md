# QuickStep

[AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) is a library for Arduino that allows you to control stepper motors, but is has flaws.

QuickStep is an alternative that:
- is x% faster than AccelStepper (see picture)
- Includes acceleration when running multiple motors that has to stop at the same time.
- Can do interpolation to create smooth paths.
- You can run one motor at one speed while performing actions with the others. At EasySort this allows us to do pickup from a moving conveyor.

## How to use
Download the .zip file and import it in the Arduino IDE. For instructions see [this arduino page](https://support.arduino.cc/hc/en-us/articles/5145457742236-Add-libraries-to-Arduino-IDE)