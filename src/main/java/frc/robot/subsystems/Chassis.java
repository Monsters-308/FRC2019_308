/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.teleop_drive;
import frc.robot.RobotMap;
import frc.robot.OI;

/**
 * Add your docs here.
 */
public class Chassis extends PIDSubsystem {

  //import motor objects from RobotMap
 public static WPI_TalonSRX leftFront = RobotMap.leftFrontMotor;
 public static WPI_TalonSRX rightFront = RobotMap.rightFrontMotor;
 public static WPI_TalonSRX leftBack = RobotMap.leftBackMotor;
 public static WPI_TalonSRX rightBack = RobotMap.rightBackMotor;

  //"Arcade Drive" object
  public static DifferentialDrive mainDrive = new DifferentialDrive(leftFront, rightFront);

  //store joystick values to pass to mainDrive
  public static double forward;
  public static double reverse;
  public static double turn;
  public static double turnCoef;
  public static double driveCoef;

  /**
   * Add your docs here.
   */
  public Chassis() {
    // Insert a subsystem name and PID values here
    super("Chassis", 1, 0.2, 0.3);
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

    leftFront.setNeutralMode(NeutralMode.Brake);
    rightFront.setNeutralMode(NeutralMode.Brake);
    leftBack.setNeutralMode(NeutralMode.Brake);
    rightBack.setNeutralMode(NeutralMode.Brake);

    turnCoef = 0.76;
    driveCoef = 1.0;

  }

  public void advancedDrive(){
    forward = Math.pow(driveCoef*OI.driveController.getRawAxis(2), 1.5);
    reverse = -Math.pow(driveCoef*OI.driveController.getRawAxis(3), 1.5);
    turn = turnCoef*OI.driveController.getRawAxis(0);

    double x = 0;

      if(forward > 0){
        x = forward;
      }else if(reverse < 0){
        x = reverse;
      }else{
        x = 0;
      }

      if(forward > 0){
        x = -forward;
      }else if(reverse < 0){
        x = -reverse;
      }else{
        x = 0;
      }

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
