package frc.robot;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SimpleElevatorSubsystem;
import frc.robot.commands.elevatorUp;
import frc.robot.commands.elevatorUpFast;
import frc.robot.commands.resetEncoder;
import frc.robot.commands.travelToSetpoint;
import frc.robot.commands.elevatorDown;
import frc.robot.commands.elevatorHold;
import frc.robot.commands.elevatorUpTwoMotors;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.commands.Cooked;
import frc.robot.commands.changePoint;
import frc.robot.commands.checkPhotoeye;
// import frc.robot.commands.coralIntake;
// import frc.robot.commands.coralReverseIntake;
import frc.robot.commands.coralPlace;
import frc.robot.commands.coralReversePlace;
import frc.robot.commands.creepMode;
import frc.robot.commands.autonCommands.IntakeCoral;
// import frc.robot.commands.autonCommands.ElevatorDownFromL4;
// import frc.robot.commands.autonCommands.ElevatorToL4;
//import frc.robot.commands.autonCommands.IntakeCoral;
import frc.robot.commands.autonCommands.SpitCoral;
import frc.robot.commands.autonCommands.elevatorUpAuton;
import frc.robot.commands.autonCommands.goToL1_from_L4;
import frc.robot.commands.autonCommands.goToPoint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.SensorSubsystem;
// import frc.robot.commands.checkSensors;

import org.json.simple.parser.ParseException;
import java.io.IOException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.Constants.Operator;

public class RobotContainer {
  CommandXboxController m_driverController = new CommandXboxController(Operator.kDriverControllerPort);
  DriveSubsystem m_drive;
  ElevatorSubsystem m_elevator;
  SimpleElevatorSubsystem elevatorSubsystem;
  CoralSubsystem m_coral;
  //SensorSubsystem m_sensor;
  SendableChooser<Command> autoChooser;

  public RobotContainer() throws IOException, ParseException{

    m_drive = new DriveSubsystem();
    m_elevator = new ElevatorSubsystem();
    elevatorSubsystem = new SimpleElevatorSubsystem();
    m_coral = new CoralSubsystem();
    //m_sensor = new SensorSubsystem();
    NamedCommands.registerCommand("toL4", new goToPoint(m_elevator,3));
    NamedCommands.registerCommand("toL3", new goToPoint(m_elevator,2));
    NamedCommands.registerCommand("toL2", new goToPoint(m_elevator,1));
    NamedCommands.registerCommand("toL1", new goToPoint(m_elevator,0));
    NamedCommands.registerCommand("shootL1", new goToPoint(m_elevator,10));
    NamedCommands.registerCommand("downFromL4", new goToL1_from_L4(m_elevator, 0));
    // NamedCommands.registerCommand("upToL4_freaky", new ElevatorToL4(m_elevator,1));
    // NamedCommands.registerCommand("upToL4_freakier", new ElevatorToL4(m_elevator,2));
    // NamedCommands.registerCommand("downFromL4_freaky", new ElevatorDownFromL4(m_elevator,1));
    // NamedCommands.registerCommand("downFromL4", new ElevatorDownFromL4(m_elevator,0));
    // NamedCommands.registerCommand("downFromL4_freakier", new ElevatorDownFromL4(m_elevator,2));
    NamedCommands.registerCommand("intakeCoral",new IntakeCoral(m_coral,m_elevator));
    NamedCommands.registerCommand("spitCoral",new SpitCoral(m_coral,m_elevator));
    NamedCommands.registerCommand("elevatorUp", new elevatorUpAuton(m_elevator, 0.3));
    NamedCommands.registerCommand("elevatorUpFast", new elevatorUpFast(m_elevator, 0.6));
    NamedCommands.registerCommand("elevatorDown", new elevatorDown(m_elevator, -0.4));
    NamedCommands.registerCommand("elevatorHold", new elevatorHold(m_elevator, 0.03));
    //NamedCommands.registerCommand("elevator Two Motor Uop", new elevatorUpTwoMotors(m_elevator, .5, .5));
    //NamedCommands.registerCommand("Coral Intake", new coralIntake(m_coral, 0.2));
    //NamedCommands.registerCommand("Coral Reverse Intake", new coralReverseIntake(m_coral, -0.2));
    NamedCommands.registerCommand("upOne", new changePoint(m_elevator, 1));
    NamedCommands.registerCommand("downOne", new changePoint(m_elevator, -1));
    NamedCommands.registerCommand("Coral Place", new coralPlace(m_coral, 0.2));
    NamedCommands.registerCommand("Coral Reverse Place", new coralReversePlace(m_coral, -0.2));
    NamedCommands.registerCommand("Creep Mode", new creepMode(m_drive));
    NamedCommands.registerCommand("Check for Photoeye", new checkPhotoeye(m_elevator));
    //NamedCommands.registerCommand("Check for Sensors", new checkSensors(m_sensor));
    
    SmartDashboard.putNumber("Elevator Up Power", 0.7);

    //SmartDashboard.putNumber("Elevator Up Power Motor 1", 0.5);
    //SmartDashboard.putNumber("Elevator Up Power Motor 2", 0.5);
    
    SmartDashboard.putNumber("Elevator Down Power", -0.35);

    m_drive.setDefaultCommand(
      m_drive.driveCommand(m_driverController::getLeftX, m_driverController::getLeftY, m_driverController::getRightX)
    );
   
    // 0.025 power up will hold both stages or just 2nd stage in place
    // 0.2 power draws <20 amps at stall
    m_elevator.setDefaultCommand(new travelToSetpoint(m_elevator));
    //m_elevator.setDefaultCommand(new checkPhotoeye(m_elevator));

    //Constantly pulling 
    //m_sensor.setDefaultCommand(new checkSensors(m_sensor));

    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser("Drive Forward");
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
      //Sets Gyro to zero where it's facing
      m_driverController.start().onTrue(m_drive.zeroGyro());

      m_driverController.rightTrigger().whileTrue(new elevatorUp(m_elevator, 0.5));
      m_driverController.rightBumper().whileTrue(new elevatorUpFast(m_elevator, 0.6));
      m_driverController.leftTrigger().whileTrue(new elevatorDown(m_elevator, -0.3));
      //m_driverController.leftBumper().whileTrue(new elevatorUpTwoMotors(m_elevator, .5, .5));
      
      // m_driverController.y().whileTrue(new coralIntake(m_coral, 0.2));
      m_driverController.a().whileTrue(new coralPlace(m_coral, 0.3));

      //Reverse coral direction
      // m_driverController.x().whileTrue(new coralReverseIntake(m_coral, -0.2));
      m_driverController.b().whileTrue(new coralReversePlace(m_coral, -0.1));

      //CHANGED TO ADD SETPOINTS
      m_driverController.a().whileTrue(elevatorSubsystem.moveToL2Command());
      m_driverController.x().whileTrue(elevatorSubsystem.moveToL3Command());
      m_driverController.y().whileTrue(elevatorSubsystem.moveToL4Command());
;
      m_driverController.povUp().onTrue(new changePoint(m_elevator,1));
      m_driverController.povDown().onTrue(new changePoint(m_elevator,-1));

      m_driverController.povLeft().onTrue(new Cooked(m_elevator));

      m_driverController.povRight().onTrue(new resetEncoder(m_elevator));

      //Creep mode means it drives slower
      //pov equals dpad
      m_driverController.leftStick().whileTrue(new creepMode(m_drive));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
   //return new ElevatorDownFromL4(m_elevator);
  }

  public DriveSubsystem getDriveSubsystem() {
    return m_drive;
  }
}