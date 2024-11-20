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

## IMPORTANT:
- If you choose to set the speeds for each motor manually in the chains, then there is no promise that the motors will stop at the same time. They will stop, when you call .procede() on all the motors, which had their speed specified.

## Todo:
The corner step should be calculated by the max speed and acceleration, which should be enough I think. 
- This would not be true if the motor is not going at full speed yet, then the calculation is off, and dynamic cornering should be used
- [x] Cornering sort of
- [x] Calculate cornering
	- [ ] This overshoot, as the motors have to end at the same time (but they do not, so this overshoot should be fixed!) (diff is most likely wrong somehow?)
- [ ] Cornering advanced (work with the full thing)
- [ ] Hold speeds
- [x] Work with changing dir !!
- [ ] Speed goes to 0??
	- Currently the length of the speeds input for the rollout is identical for with and without position3. The issue seems that somehow vol_idxs is accumilated faster, which is odd.
    - This is because of bad cornering calculation, so deleted now, and should be fixed.
- [ ] Speed plots not sure if they work with changing dir, should double check
- [ ] Maybe make extensive test to what I want it to be able to do?