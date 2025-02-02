package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import java.util.function.BooleanSupplier;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;

public class RobotContainer {
  boolean setupSwerve = true;
  CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  DriveSubsystem m_drive;
  SendableChooser<Command> autoChooser;

  public RobotContainer() {

    m_drive = new DriveSubsystem();

    autoChooser = AutoBuilder.buildAutoChooser("2 note front");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    m_drive.setDefaultCommand(
        m_drive.driveCommand(m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX));

      

    configureBindings();
  }

  private void configureBindings() {
      //Sets zero
      m_driverController.start().onTrue(m_drive.zeroGyro());
  }

  public Command getAutonomousCommand() {
    //return new PathPlannerAuto("Shoot move shoot");
    return autoChooser.getSelected();
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_drive;
  }
}