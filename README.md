# Suspension Travel & Tyre temp sensor for RaceChrono
A modified https://github.com/MagnusThome/RejsaRubberTrac that uses a potentiometer instead of a
distance sensor to measure the suspension travel. It also uses the newer MLX90640 sensor.

# Usage
1. Set the name to match the corner you are going to place each device

   ``#define DEVICE_NAME "RearLeftSensor"``
2. Place the temp sensor so that the reference pin points to the back of the car
3. Take note of the potentiometer value at ride height and at full droop. Also measure the travel from
ride height to droop. Then set the following values accordingly:

    ``#define DROOP_TRAVEL -33``

    ``#define SENSOR_RIDE_HEIGHT 398``

    ``#define SENSOR_DROOP 312``
4. Added the device as "RejsaRubberTrac" in "other devices" under RaceChrone setup.

## Safety notice
This is experimental, use it with caution and at your own risk!

## License
This is [licensed](LICENSE) under the [MIT Licence](http://en.wikipedia.org/wiki/MIT_License).
