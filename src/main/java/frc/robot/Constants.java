// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

//This is Phys Properties Varibles
//"drive": 0.047286787200699704,
//"angle": 16.8


public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double driveDeadband = 0.05;
  }

  public class NonChassisConstants {
    public static int elevatorMotorID1 = 12;
    public static int elevatorMotorID2 = 14;
    public static int intakeMotorID = 16;    
  }

  public static class ModuleConstants {
        // The max free speed of the module
        public static final double maxSpeed = 4.5;
        //Lowerd from 4.5

        public static double trackWidth = 0.5588;
        public static double wheelBase = 0.4826;

        public static Translation2d frontLeftPosition = new Translation2d(trackWidth/2,wheelBase/2);
        public static Translation2d frontRightPosition = new Translation2d(-trackWidth/2,wheelBase/2);
        public static Translation2d backLeftPosition = new Translation2d(trackWidth/2,-wheelBase/2);
        public static Translation2d backRightPosition = new Translation2d(-trackWidth/2,-wheelBase/2);

        public static SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(frontLeftPosition,
                                                                                frontRightPosition,
                                                                                backLeftPosition,
                                                                                backRightPosition);

  }
}
