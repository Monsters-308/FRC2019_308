/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.OI;
import frc.robot.RobotMap;
import frc.robot.commands.teleop_cargodelivery;

/**
 * Add your docs here.
 */
public class CargoDelivery extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static WPI_TalonSRX cargoIntakeMotor = RobotMap.cargoIntakeMotor;
  public static Solenoid deliveryArmSolenoid;
  public static boolean isRaised;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new teleop_cargodelivery());
  }

  public void setupCargoDelivery(){
    deliveryArmSolenoid = RobotMap.deliveryArmSolenoid;
    isRaised = true;
    deliveryArmSolenoid.set(false);
  }

  public void controlCargoDelivery(){
    if(isRaised == true && OI.operator.getRawButton(3) == true){
      deliveryArmSolenoid.set(true);
      isRaised = false;
    }else if(isRaised == false && OI.operator.getRawButton(3) == true){
      deliveryArmSolenoid.set(false);
      isRaised = true;
    }

    if(OI.operator.getRawButton(7) == true){
      cargoIntakeMotor.set(1.0);
    }else if(OI.operator.getRawButton(8) == true){
      cargoIntakeMotor.set(-1.0);
    }else{
      cargoIntakeMotor.set(0.0);
    }
  }
}
