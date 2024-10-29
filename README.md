# QuickStep

AccelStepper is a library for Arduino that allows you to control stepper motors, but is has flaws.

QuickStep is an alternative that:
- is x% faster than AccelStepper (see picture)
- Includes acceleration when running multiple motors that has to stop at the same time.
- Can do interpolation to create smooth paths.
- You can run one motor at one speed while performing actions with the others. At EasySort this allows us to do pickup from a moving conveyor.