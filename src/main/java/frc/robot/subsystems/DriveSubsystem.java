// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
public class DriveSubsystem extends SubsystemBase {

  /* 
  !!! DOCUMENTATION I USED TO MAKE THIS !!!
  https://yagsl.gitbook.io/yagsl/configuring-yagsl/code-setup
  https://yagsl.gitbook.io/yagsl/bringing-up-swerve/creating-your-first-configuration
  https://github.com/BroncBotz3481/YAGSL-Configs/tree/main

  EDIT CAN IDS AND OTHER MODULE CONSTANTS IN THE DEPLOY DIRECTORY (YAGSL-Test\src\main\deploy\swerve\modules)
  
  */ 
  
  public SwerveDrive swerveDrive;

  SwerveDriveOdometry m_odometry;
  Field2d m_field = new Field2d();

  
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    File swerveJsons = new File(Filesystem.getDeployDirectory(), "swerve");

    zeroGyro();

  SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    
    try {
      swerveDrive = new SwerveParser(swerveJsons).createSwerveDrive(ModuleConstants.maxSpeed);
    }
    catch (IOException exception) {
      DriverStation.reportError("Error with loading config: ", exception.getStackTrace());
    }

    AutoBuilder.configureHolonomic(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                    new PIDConstants(4, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(4, 0.0, 0.0), // Rotation PID constants
                    ModuleConstants.maxSpeed, // Max module speed, in m/s
                    0.356, // Drive base radius in meters. Distance from robot center to furthest module.
                
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
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
    SmartDashboard.putData("Field",m_field);
  }

  /**
   * Command to drive the robot using translative values and heading as angular velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param tran
   * 
   * slationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      double x = -MathUtil.applyDeadband(translationX.getAsDouble(), OperatorConstants.driveDeadband);
      x = Math.pow(x, 3);
      double y = -MathUtil.applyDeadband(translationY.getAsDouble(), OperatorConstants.driveDeadband);
      y = Math.pow(y, 3);
      double angle = -MathUtil.applyDeadband(angularRotationX.getAsDouble(), OperatorConstants.driveDeadband);
      angle = Math.pow(angle,3);
      swerveDrive.drive(new Translation2d(y * swerveDrive.getMaximumVelocity()/4,
                        x * swerveDrive.getMaximumVelocity()/4),
                        angle * swerveDrive.getMaximumAngularVelocity()/2,
                        true,
                        false);
    });
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_field.setRobotPose(getPose());
  }

  public Command zeroGyro() {
    return new InstantCommand(() ->
    swerveDrive.zeroGyro());
  }

  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  public void resetPose(Pose2d initialHolonomicPose)
  {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getRobotRelativeSpeeds()
  {
    return swerveDrive.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds velocity)
  {
    swerveDrive.drive(velocity);
  }
}
