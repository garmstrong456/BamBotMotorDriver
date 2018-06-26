# Arduino library for the Pololu DRV8835 Dual Motor Driver Shield
Based on the Pololu DRV8835MotorShield library
Customized for the Idea7 Bambot by Greg Armstrong, June 2018
See pololu.com for the original code

## Function reference
### Motor functions:
##### init(M1Pwm, M1Dir, M2Pwm, M2Dir)
Initialize the motor object assuming all four pins are connected directly to the M5Stack
##### init(mcp, M1Pwm, M1Dir, M2Pwm, M2Dir)
Initialize the motors assuming the PWM pins are connected to the M5 and the direction pins are connected to the MCP23008. "mcp" is an "Adafruit_MCP23008" object. The MCP must be initialized before calling this function.
##### setM1Speed(speed), setM2Speed(speed)
Set the speed for one motor at a time. "speed" must be an integer between -400 and 400
##### setSpeeds(speed1, speed2)
Set the speed for both motors at once
##### flipM1(boolean), flipM2(boolean)
Set the reverse flag for a motor

### Encoder functions:
##### attachEncoders(pin1A, pin1B, pin2A, pin2B)
Initialize the encoders using the pins provided
##### motor1Position(), motor2Position()
Returns the current position of motor 1 or 2
##### motor1ResetPosition(), motor2ResetPosition()
Sets motor 1 or 2 position to 0
