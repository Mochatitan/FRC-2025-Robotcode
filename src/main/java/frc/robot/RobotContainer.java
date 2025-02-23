package frc.robot;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.elevatorUp;
import frc.robot.commands.elevatorDown;
import frc.robot.commands.elevatorHold;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.commands.coralIntake;
import frc.robot.commands.coralOuttake;
import frc.robot.commands.coralPlace;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.json.simple.parser.ParseException;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import frc.robot.Constants;
import frc.robot.Constants.Operator;

public class RobotContainer {
  CommandXboxController m_driverController = new CommandXboxController(Operator.kDriverControllerPort);
  DriveSubsystem m_drive;
  ElevatorSubsystem m_elevator;
  CoralSubsystem m_coral;
  SendableChooser<Command> autoChooser;

  public RobotContainer() throws IOException, ParseException{

    m_drive = new DriveSubsystem();
    m_elevator = new ElevatorSubsystem();
    m_coral = new CoralSubsystem();
    NamedCommands.registerCommand("Elevator Up", new elevatorUp(m_elevator, 0.2));
    NamedCommands.registerCommand("Elevator Down", new elevatorDown(m_elevator, -0.2));
    NamedCommands.registerCommand("Elevator Hold", new elevatorHold(m_elevator, 0.025));
    NamedCommands.registerCommand("Coral Intake", new coralIntake(m_coral, 0.2));
    NamedCommands.registerCommand("Coral Intake", new coralOuttake(m_coral, -0.2));
    NamedCommands.registerCommand("Coral Place", new coralPlace(m_coral, -0.2));
   
    autoChooser = AutoBuilder.buildAutoChooser("Drive Foward");
    SmartDashboard.putData("Auto Chooser", autoChooser);

    
    m_drive.setDefaultCommand(
      m_drive.driveCommand(m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX)
    );
   
    // 0.025 power up will hold both stages or just 2nd stage in place
    // 0.2 power draws <20 amps at stall
    m_elevator.setDefaultCommand(
    new elevatorHold(m_elevator, 0.025)
    );

    configureBindings();
  }

  private void configureBindings() {
      //Sets Gyro to zero where it's facing
      m_driverController.start().onTrue(m_drive.zeroGyro());

      m_driverController.rightTrigger().whileTrue(new elevatorUp(m_elevator, 0.1));
      m_driverController.leftTrigger().whileTrue(new elevatorDown(m_elevator, -0.1));
      m_driverController.leftBumper().whileTrue(new coralIntake(m_coral, 0.2));
      m_driverController.y().whileTrue(new coralOuttake(m_coral, -0.2));
      m_driverController.rightBumper().whileTrue(new coralPlace(m_coral, -0.2));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_drive;
  }
}