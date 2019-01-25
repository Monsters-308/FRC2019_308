/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.teleop_drive;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.OI;

/**
 * Add your docs here.
 */
public class Chassis extends PIDSubsystem {

 public static WPI_TalonSRX leftFront = RobotMap.leftFrontMotor;
 public static WPI_TalonSRX rightFront = RobotMap.rightFrontMotor;
 public static WPI_TalonSRX leftBack = RobotMap.leftBackMotor;
 public static WPI_TalonSRX rightBack = RobotMap.rightBackMotor;

  public static DifferentialDrive mainDrive = new DifferentialDrive(leftFront, rightFront);

  public static double forward;
  public static double reverse;
  public static double turn;

  public static boolean isHatchDirection = true;

  /**
   * Add your docs here.
   */
  public Chassis() {
    // Insert a subsystem name and PID values here
    super("Chassis", 1, 0.2, 0.3);
    //setAbsoluteTolerance(0.05);
    //getPIDController().setContinuous(true);
    //getPIDController().setSetpoint(forward);
    //getPIDController().enable();
    //Shortcut push test
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new teleop_drive());
  }

  public void setupdrive(){

    leftBack.follow(leftFront);
    rightBack.follow(rightFront);

    leftFront.setInverted(false);
    rightFront.setInverted(false);
    leftBack.setInverted(false);
    rightBack.setInverted(false);

    // leftBack.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
  }
  

  public void drive(){
    //RobotMap.compressor.

    forward = OI.driveController.getRawAxis(2);
    reverse = -OI.driveController.getRawAxis(3);
    turn = OI.driveController.getRawAxis(4);

    double x = 0;

    if(OI.driveController.getRawButton(4) == true && isHatchDirection == false){
      isHatchDirection = true;
    }else if (OI.driveController.getRawButton(1) == true && isHatchDirection == true){
      isHatchDirection = false;
    }

    //if(isHatchDirection == true){
      if(forward > 0){
        x = forward;
      }else if(reverse < 0){
        x = reverse;
      }else{
        x = 0;
      }
   // }else if(isHatchDirection == false){
      if(forward > 0){
        x = -forward;
      }else if(reverse < 0){
        x = -reverse;
      }else{
        x = 0;
      }


    //}
    mainDrive.arcadeDrive(x, turn);
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // return leftFront.getSelectedSensorPosition();
    return 0.0;
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // leftFront.pidWrite(output);
    // rightFront.pidWrite(output);
  }
}
