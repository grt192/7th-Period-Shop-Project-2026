// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TankDriveSubsystem extends SubsystemBase {
  
  private WPI_TalonSRX lF = new WPI_TalonSRX(2);
  private WPI_TalonSRX lB = new WPI_TalonSRX(1);
  private WPI_TalonSRX rF = new WPI_TalonSRX(18);
  private WPI_TalonSRX rB = new WPI_TalonSRX(4);
  //replace the 'a' with numbers

  public TankDriveSubsystem() {

    lB.setNeutralMode(NeutralMode.Coast);
    rB.setNeutralMode(NeutralMode.Coast);
    lF.setNeutralMode(NeutralMode.Coast);
    rF.setNeutralMode(NeutralMode.Coast);
    lB.follow(lF);
    rB.follow(rF);

  }

  public void setMotors(double left, double right){
    rF.set(right);
    lF.set(-left);
  }

  public void setBrakeMode(boolean b){
    rF.setNeutralMode(NeutralMode.Brake);
    lF.setNeutralMode(NeutralMode.Brake);
  }

  public void altDrive(double lA, double rA){
    if (lA == 0 && rA != 0) {
      // Spin in place
      setMotors(rA, -rA);
    } else if (lA != 0 && rA == 0) {
      // Move straight
      setMotors(lA, lA);
    } else {
      // mix
      setMotors((lA - rA) / 2.0, (lA + rA) / 2.0);
    }
  }

  //these methods were more or less taken from the other tankDrive system. The specific functions were copied too, as I didnt
  //feel like searching for and reading documentation.
  
}
