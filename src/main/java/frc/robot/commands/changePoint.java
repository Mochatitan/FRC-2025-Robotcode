// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class changePoint extends InstantCommand {
  private ElevatorSubsystem m_elevator;
  private int m_toChange;
  public changePoint(ElevatorSubsystem elevator, int toChange) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elevator = elevator;
    m_toChange = toChange;
    addRequirements(m_elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setTarget(m_elevator.getTarget() + m_toChange);
  }
}
