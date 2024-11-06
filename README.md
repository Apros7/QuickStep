# QuickStep

Before you read: Yes this code is shit. Yes I know it is. I made it in a hurry and it is not optimized. But it works. For now. Use it if you want :D.

[AccelStepper](https://www.airspayce.com/mikem/arduino/AccelStepper/) is a library for Arduino that allows you to control stepper motors, but is has flaws.

QuickStep is an alternative that:
- is x% faster than AccelStepper (see picture)
- **Includes acceleration when running multiple motors that has to stop at the same time.**
- Can do interpolation to create smooth paths.
- You can run one motor at one speed while performing actions with the others. At EasySort this allows us to do pickup from a moving conveyor.

The code is under the [MIT Licence](LICENSE), so feel free to use and modify to fit your needs.

## How to use
Download the .zip file and import it in the Arduino IDE. For instructions see [this arduino page](https://support.arduino.cc/hc/en-us/articles/5145457742236-Add-libraries-to-Arduino-IDE)

## Todo:
- [x] What if Accel is too slow then should deaccel
- [ ] What if multiple motors
- [x] What if uneven steps
- What is motors can have different max speed and acceleration?
- What if one motor has to have a permanent speed while others are moving
- Z axis should change because it depends on x, and it should be able to handle this.
- Second corner now is pretty bad