/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static Compressor compressor = new Compressor();
  public static Solenoid gripperSolenoid = new Solenoid(0);

  //Motors
    //Drive Motors
  public static WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(0);
  public static WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(1);
  public static WPI_TalonSRX leftBackMotor = new WPI_TalonSRX(2);
  public static WPI_TalonSRX rightBackMotor = new WPI_TalonSRX(3);

    //Intake Motors

    //Delivery mechanism motors

  //Sensors
    //Chassis sensors

    //Gripper sensors

  public static DigitalInput gripperSwitch1 = new DigitalInput(0);
  public static DigitalInput gripperSwitch2 = new DigitalInput(1);
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}