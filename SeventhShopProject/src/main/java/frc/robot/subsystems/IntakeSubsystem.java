// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;


//change auto to use setposition instead of velocity
//manual not necessary


public class IntakeSubsystem extends SubsystemBase {
  
  private TalonFX leverMotor = new TalonFX( /*insert numer */ 3);
  private CANdi limit = new CANdi(/*insert number */ 2);
  private boolean autoOn = false;
  private double upperLim = /*decide val */ 3.5; //check movearm to change value, 50 is just exorbitantly large random number, but check signage here
  private double invert = 1; //to reverse direction, just change 1 to -1
  DoublePublisher pos;
  TalonFXConfiguration PID = new TalonFXConfiguration();
  
  

  
  public IntakeSubsystem() {
    leverMotor.setPosition(0.0);
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("data");
    pos = table.getDoubleTopic("pos").publish();
    config();
    
  }

  private void config(){
    PID.Slot0.kP = 0;                                                //fix these guys somehow
    PID.Slot0.kI = 1;
    PID.Slot0.kV = 2;
    PID.Slot0.kD = 3;
    PID.Slot0.kG = 4;
    leverMotor.getConfigurator().apply(PID);
  }

  public void manual(boolean left, boolean right){
    if(left && right){
      leverMotor.set(0);
      leverMotor.setControl(new PositionVoltage(leverMotor.getPosition().getValueAsDouble()));
      autoOn = true;
      return;
    }

    if(left && !right ){
      leverMotor.set(-0.1*invert);
    }else if(!left && right){
      leverMotor.set(0.1*invert);
    }else{
      leverMotor.set(0.0*invert);
      leverMotor.setControl(new PositionVoltage(leverMotor.getPosition().getValueAsDouble()));
    }
  }

  public void autoSetIntake(boolean left, boolean right){

    if(left && right){
      leverMotor.set(0);
      leverMotor.setControl(new PositionVoltage(leverMotor.getPosition().getValueAsDouble()));
      autoOn = false;
      return;
    }

    if(left && !right ){
      if(leverMotor.getPosition().getValueAsDouble() > -0.5){// check sign on upperLim
        leverMotor.set(-0.1*invert);
      }else{
        leverMotor.setControl(new PositionVoltage(0.0));
      }
    }else if(!left && right){
        leverMotor.setControl(new PositionVoltage(0.0));  
    }

  }

  public void moveArm(boolean left, boolean right){

    pos.set(leverMotor.getPosition().getValueAsDouble());

    if(!autoOn){
      manual(left, right);
    }

    if(limit.getS1Closed().refresh().getValue()){ //check is bool is true or false when pressed
      leverMotor.set(0.0);
      leverMotor.setPosition(0.0); 
      leverMotor.setControl(new PositionVoltage(0.0));
    }

    if(leverMotor.getPosition().getValueAsDouble() > upperLim){//check signage
      leverMotor.set(0.0);
      leverMotor.setControl(new PositionVoltage(3.5));
    }

    if(autoOn){
      autoSetIntake(left, right);
    }

  }


}
