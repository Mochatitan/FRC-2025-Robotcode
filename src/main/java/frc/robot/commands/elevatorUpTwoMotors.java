// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class elevatorUpTwoMotors extends Command {
  private ElevatorSubsystem m_elevator;
  private double m_power1;
  private double m_power2;
  public elevatorUpTwoMotors(ElevatorSubsystem elevator, double power1, double power2) {
    m_elevator = elevator;
    m_power1 = power1;
    m_power2 = power2;
    addRequirements(m_elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power1 = SmartDashboard.getNumber("Elevator Up Power Motor 1: ", m_power1);
    double power2 = SmartDashboard.getNumber("Elevator Up Power Motor 2: ", m_power2);
    m_elevator.elevatorTwoMotorUp(power1, power2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.elevatorTwoMotorUp(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}