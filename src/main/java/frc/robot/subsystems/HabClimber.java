/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.subsystems.*;
import frc.robot.RobotMap;
import frc.robot.commands.teleop_habclimber;

/**
 * Add your docs here.
 */
public class HabClimber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static Solenoid frontClimberSolenoid;
  public static Solenoid backClimberSolenoid;
  public static boolean isArmed;
  public static boolean frontDown;
  public static boolean backDown;


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new teleop_habclimber());
  }

  public void setupHabClimber(){
    frontClimberSolenoid = RobotMap.frontClimberSolenoid;
    backClimberSolenoid = RobotMap.backClimberSolenoid;
    isArmed = false;
    frontDown = false;
    backDown = false;
  }

  public void controlClimber(){
    if(OI.operator.getRawButton(10) == true){
      isArmed = true;
      Chassis.driveCoef = 0.85;
      Chassis.turnCoef = 0.65;
    }

    if(OI.driveController.getRawButton(2) == true && isArmed == true){
      frontClimberSolenoid.set(true);
    }else{
      frontClimberSolenoid.set(false);
    }

    if(OI.driveController.getRawButton(4) == true && isArmed == true){
      backClimberSolenoid.set(true);
    }else{
      backClimberSolenoid.set(false);
    }
  }
}
