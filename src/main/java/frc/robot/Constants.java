// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

  public static class ModuleConstants {
    public static final double wheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double wheelRadius = Units.inchesToMeters(wheelDiameterMeters);

    public static final double drivingMotorReduction = 5.14; 
        
    public static final double turningMotorReduction = 41.25; 

    public static final double driveWheelFreeSpeedRps = (5676.0 / 60.0) / drivingMotorReduction;

    public static final boolean turningEncoderInverted = true;
        
    public static final int drivingMotorCurrentLimit = 40;
    public static final int turningMotorCurrentLimit = 20;
  }

  public static class DriveContants {
    public static final int flDriveCanID = 3;
    public static final int flAngleCanID = 4;

    public static final int frDriveCanID = 1;
    public static final int frAngleCanID = 2;

    public static final int blDriveCanID = 5;
    public static final int blAngleCanID = 6;

    public static final int brDriveCanID = 7;
    public static final int brAngleCanID = 8;

    public static final double mass = 13.0;
    public static final double MOI = 5.0;

    public static final double moduleOffsetMeters = Units.inchesToMeters(26.06) / 2.0;

  /*
    
    public static final List<Translation2d> moduleOffsets = List.of(
        new Translation2d(moduleOffsetMeters, moduleOffsetMeters),   // Front Left (+X, +Y)
        new Translation2d(moduleOffsetMeters, -moduleOffsetMeters),  // Front Right (+X, -Y)
        new Translation2d(-moduleOffsetMeters, moduleOffsetMeters),  // Back Left (-X, +Y)
        new Translation2d(-moduleOffsetMeters, -moduleOffsetMeters)  // Back Right (-X, -Y)
    );

  */


    public static final double flChassisAngularOffset = Math.toRadians(274);
    public static final double frChassisAngularOffset = Math.toRadians(250);
    public static final double blChassisAngularOffset = Math.toRadians(65);
    public static final double brChassisAngularOffset = Math.toRadians(72);

    public static final double trackWidth = Units.inchesToMeters(26.06); // Sağ-sol tekerlek arası
    public static final double wheelBase = Units.inchesToMeters(21.34);  // Ön-arka tekerlek arası

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2),   // Front Left
            new Translation2d(wheelBase / 2, -trackWidth / 2),  // Front Right
            new Translation2d(-wheelBase / 2, trackWidth / 2),  // Rear Left
            new Translation2d(-wheelBase / 2, -trackWidth / 2)  // Rear Right
        );

    public static final double maxSpeedMetersPerSecond = 10;
    public static final double maxAngularSpeed = (10 * Math.PI);


    
    public static final boolean gyroReversed = true;
  }

  public static final class OIConstants {
    public static final int primaryPort = 0;

    // 0.05 ile 0.1 arası 
    public static final double driveDeadband = 0.1;
  }

  public static final class ShooterConstants {
    public static final int masterNeoCanID = 11;
    public static final int follower1NeoCanID = 9;
    public static final int follower2NeoCanID = 10;

    public static final int hoodMotorPWM = 1;
    public static final int hoodPotPort = 2;
    public static final int hoodPotRange = 270;
    public static final double hoodPotOffset = 28.5;

  }

  public static final class FeederConstants {
    public static final int indexerMotorPWM = 2;

  }

  public static final class IntakeConstants {
    public static final int intakeMotorPWM = 0;
    public static final int intakeForwardChannel = 0;
    public static final int intakeReverseChannel = 1;
  }

  public static final class ClimbConstants {
    public static final int climbForwardChannel = 2;
    public static final int climbReverseChannel = 3;
  }

  public static final class targetLock {
    public static final double targetXBlue = 4.625594;
    public static final double targetYBlue = 4.034536;
    public static final double targetXRed = 11.898000;
    public static final double targetYRed = 4.013000;
  }

  public static final class LedConstants {
    public static final int kLEDDioPort = 0;
    public static final int kLedLenght = 8;
  }
}
