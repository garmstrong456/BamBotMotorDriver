#include <BamBotMotorDriver.h>
#include <M5Stack.h>

BamBotMotorDriver motors;

/*
 * Note there are two possible default wiring setups:
 * 
 * With MCP23008:
 * EN_A --> M5Stack GPIO 3/R0 (speed pin)
 * EN_B --> M5Stack GPIO 1/T0 (speed pin)
 * PH_A --> MCP 7 (direction pin)
 * PH_B --> MCP 6 (direction pin)
 * 
 * Without MCP23008:
 * EN_A --> M5Stack GPIO 3/R0  (speed pin)
 * EN_B --> M5Stack GPIO 17/R2 (speed pin)
 * PH_A --> M5Stack GPIO 1/T0  (direction pin)
 * PH_B --> M5Stack GPIO 16/T2 (direction pin)
 * 
 * See the other example for guidelines on how to initialize motors using the MCP
 */


void setup() {
  //Always call this first to avoid problems
  M5.begin();
  
  //to change pins use:
  //motors.init(M1_enable, M1_dir, M2_enable, M2_dir);
  motors.init();

  //Uncomment these to reverse one or both motors
  //motors.flipM1(true);
  //motors.flipM2(true);
}

//Drive back and forth
//Use this to check and see if one or both of your motors need to be reversed
void loop() {
  motors.setSpeeds(300, 300);
  M5.Lcd.setCursor(0,0,2); M5.Lcd.print("Forward");
  delay(1000);
  motors.setM1Speed(-300);
  motors.setM2Speed(-300);
  M5.Lcd.setCursor(0,0,2); M5.Lcd.print("Reverse");
  delay(1000);
}
