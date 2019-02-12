/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  //Motors:

    //Drive Motors
  public static WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(0);
  public static WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(1);
  public static WPI_TalonSRX leftBackMotor = new WPI_TalonSRX(2);
  public static WPI_TalonSRX rightBackMotor = new WPI_TalonSRX(3);

    //Cargo Delivery Motors
  public static WPI_TalonSRX cargoIntakeMotor  = new WPI_TalonSRX(4);

    //Lift Motors
  public static WPI_TalonSRX liftMotor1 = new WPI_TalonSRX(5);
  public static WPI_TalonSRX liftMotor2 = new WPI_TalonSRX(6);
//////////////////////////////////////////////////////////////////////////////////////////////

  //Sensors:
  
    //Chassis Sensors
  public static ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    //Gripper Sensors
  public static DigitalInput gripperSwitch1 = new DigitalInput(0);
  public static DigitalInput gripperSwitch2 = new DigitalInput(1);

    //Lift Sensors
  public static DigitalInput bottomLiftSwitch = new DigitalInput(2);
  public static DigitalInput middleLiftSwitch = new DigitalInput(3);
  public static DigitalInput topLiftSwitch = new DigitalInput(4);
//////////////////////////////////////////////////////////////////////////////////////////////

  //Pnuematics:
  public static Compressor compressor = new Compressor(7);

    //Gripper Solenoids
  public static Solenoid gripperSolenoid = new Solenoid(7,0); 

    //Hab Climber Solenoids
  public static Solenoid frontClimberSolenoid = new Solenoid(7,1);
  public static Solenoid backClimberSolenoid = new Solenoid(7,3);

    //Cargo Delivery Solenoids
  public static Solenoid deliveryArmSolenoid = new Solenoid(7,2);
//////////////////////////////////////////////////////////////////////////////////////////////      

}