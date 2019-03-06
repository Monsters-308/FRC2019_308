/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * Add your docs here.
 */
public class Lights extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private static I2C i2c = new I2C(Port.kOnboard, 4);
  private static int _currentLEDState = 0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    _currentLEDState = 0;
  }

  public static void setState(int state){
    if(_currentLEDState != state){
      i2c.write(1,(byte)state);
      _currentLEDState = state;
    }
  }
}
