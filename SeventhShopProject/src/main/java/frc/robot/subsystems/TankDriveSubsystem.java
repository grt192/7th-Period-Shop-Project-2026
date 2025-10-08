// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TankDriveSubsystem extends SubsystemBase {
  
  private WPI_TalonSRX lF = WPI_TalonSRX(a);
  private WPI_TalonSRX lB = WPI_TalonSRX(a);
  private WPI_TalonSRX rF = WPI_TalonSRX(a);
  private WPI_TalonSRX rB = WPI_TalonSRX(a);
  //replace the 'a' with numbers

  public TankDriveSubsystem() {

    lB.coast(NeutralMode.Coast);
    rB.coast(NeutralMode.Coast);
    lF.coast(NeutralMode.Coast);
    rF.coast(NeutralMode.Coast);
    lB.follow(lF);
    rB.follow(rF);

  }

  public void setMotors(double left, double right){
    rF.set(right);
    lF.set(-left);
  }

  public void brake(){
    rF.setNeutralMode(NeutralMode.Brake);
    lF.setNeutralMode(NeutralMode.Brake);
  }

  //these methods were more or less taken from the other tankDrive system. The specific functions were copied too, as I didnt
  //feel like searching for and reading documentation.
  
}
