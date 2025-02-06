// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.NonChassisConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class CoralSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public SparkMax intakeMotor = new SparkMax(NonChassisConstants.intakeMotorID, MotorType.kBrushless);

  public CoralSubsystem() {
  
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void coralIntake(double power) {
    intakeMotor.set(power);
  }

  public void coralPlace(double power) {
    intakeMotor.set(power);
  }
}
