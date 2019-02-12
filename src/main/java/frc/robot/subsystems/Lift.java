/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import frc.robot.RobotMap;
import frc.robot.commands.teleop_lift;
import frc.robot.OI;

/**
 * Add your docs here.
 */
public class Lift extends PIDSubsystem {
  
  //import motor objects from RobotMap
public static WPI_TalonSRX liftMotor1 = RobotMap.liftMotor1;
public static WPI_TalonSRX liftMotor2 = RobotMap.liftMotor2;

  //create local limit switch objects
public static DigitalInput bottomLiftSwitch;
public static DigitalInput middleLiftSwitch;
public static DigitalInput topLiftSwitch;  
public static boolean isDown;
public static boolean isMid;
public static boolean isTop;
public static int targetState;
public static int liftState;
  
  public Lift() {
    // Intert a subsystem name and PID values here
    super("SubsystemName", 1, 2, 3);
    // Use these to get going:
    // setSetpoint() - Sets where the PID controller should move the system
    // to
    // enable() - Enables the PID controller.
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new teleop_lift());
  }

  public void setupLift(){
    topLiftSwitch = RobotMap.topLiftSwitch;
    middleLiftSwitch = RobotMap.middleLiftSwitch;
    bottomLiftSwitch = RobotMap.bottomLiftSwitch;

    liftMotor2.follow(liftMotor1);
    liftMotor2.setInverted(false);
    liftMotor1.setInverted(true);

    isDown = true;
    isMid = false;
    isTop = false;

    targetState = 0;
    liftState = 0;
  }

public void getLiftStates(){
  if(bottomLiftSwitch.get() == false){
    isDown = true;
    liftState = 0;
  }else{
    isDown = false;
  }

if(bottomLiftSwitch.get() == false){
  isMid = true;
  liftState = 1;
}else{
  isMid = false;
}

  if(topLiftSwitch.get() == false){
    isTop = true;
    liftState = 2;
  }else{
    isTop = false;
  }
}

public void setTargetState(){
  if(OI.operator.getRawButton(4) == true){
    targetState = 2;
  }else if(OI.operator.getRawButton(3) == true){
    targetState = 1;
  }else if(OI.operator.getRawButton(1) == true){
    targetState = 0;
  }
}

  public void basicLiftControl(){
     if(OI.operator.getRawButton(4) == true){
       liftMotor1.set(1.0);
     }else if(OI.operator.getRawButton(2) == true){
      liftMotor1.set(-1.0);
     }else{
       liftMotor1.set(0.0);
     }
  }

  public void advancedLiftControl(){
    if(targetState > liftState){
      liftMotor1.set(1.0);
    }else if(targetState < liftState){
      liftMotor1.set(-1.0);
    }else if(targetState == liftState){
      liftMotor1.set(0.0);
    }else if(targetState == 3 && liftState == 3 && isTop == false){
      liftMotor1.set(1.0);
    }else if(targetState == 2 && liftState == 2 && isMid == false && liftMotor1.get() > 0){
      liftMotor1.set(-1.0);
    }else if(targetState == 2 && liftState == 2 && isMid == false && liftMotor1.get() < 0){
      liftMotor1.set(1.0);
    }else  if(targetState == 1 && liftState == 1 && isDown == false){
      liftMotor1.set(-1.0);
    }
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return 0.0;
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
  }
}
