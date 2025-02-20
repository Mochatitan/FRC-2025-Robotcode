// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.auto.CommandUtil;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Operator;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class DriveSubsystem extends SubsystemBase {

  public SwerveDrive swerveDrive;

  SwerveDriveOdometry m_odometry;
  Field2d m_field = new Field2d();


  public DriveSubsystem() throws IOException, ParseException{
    // All other subsystem initialization
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config = RobotConfig.fromGUISettings();

    // Configure AutoBuilder last
    AutoBuilder.configure(
      this::getPose, // Robot pose supplier
      this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
        new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }
        
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      double x = -MathUtil.applyDeadband(translationX.getAsDouble(), Operator.driveDeadband);
      x = Math.pow(x, 3);
      double y = -MathUtil.applyDeadband(translationY.getAsDouble(), Operator.driveDeadband);
      y = Math.pow(y, 3);
      double angle = -MathUtil.applyDeadband(angularRotationX.getAsDouble(), Operator.driveDeadband);
      angle = Math.pow(angle,3);
      swerveDrive.drive(new Translation2d(y * swerveDrive.getMaximumChassisVelocity()/4,
                        x * swerveDrive.getMaximumChassisVelocity()/4),
                        angle * swerveDrive.getMaximumChassisAngularVelocity()/2,
                        true,
                        false);
    });
  };


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_field.setRobotPose(getPose());
  }

  public Command zeroGyro() {
    return new InstantCommand(() ->
    this.swerveDrive.zeroGyro());
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d initialHolonomicPose)
  {
    this.swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    return this.swerveDrive.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds velocity)
  {
    this.swerveDrive.drive(velocity);
  }
}
