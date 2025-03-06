// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {
  public SparkMax elevatorMotor1 = new SparkMax(Constants.NonChassis.elevatorMotorID1, MotorType.kBrushless);
  public SparkMax elevatorMotor2 = new SparkMax(Constants.NonChassis.elevatorMotorID2, MotorType.kBrushless);

  DigitalInput input = new DigitalInput(1);

  public ElevatorSubsystem() {
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void elevatorUp(double power) {
    if (input.get()){
    elevatorMotor1.set(power);
    elevatorMotor2.set(-power);
    System.out.println("Elevator Up Power: " + power);
    }
    else {
    System.out.println("Elevator Blocked By Coral");
    }
  }

  public void elevatorUpFast(double power) {
    if (input.get()){
    elevatorMotor1.set(power+.5);
    elevatorMotor2.set(-power-.5);
    System.out.println("Elevator Up Power: " + power);
    }
    else {
    System.out.println("Elevator Blocked By Coral");
    }
  }

  public void elevatorDown(double power) {
    elevatorMotor1.set(power);
    elevatorMotor2.set(-power);
    System.out.println("Elevator Down Power: " + power);
  }

  public void elevatorHold(double power)
  {
    elevatorMotor1.set(power);
    elevatorMotor2.set(-power);
    //System.out.println("Elevator Hold Power: " + power);
  }

  public void elevatorStop(double power)
  {
    elevatorMotor1.set(power);
    elevatorMotor2.set(-power);
  }

  public boolean checkPhotoeye()
  {
    SmartDashboard.putBoolean("Photoeye",input.get());
    return input.get();
  }
}
