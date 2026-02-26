// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.Utils.SwerveModuleConstants;

/** Add your docs here. */
public class Constants {

    public static class OperatorConstants {
    public static final int driverController = 0;
    public static final int operatorController = 1;
  }

  public static class shooterConstants {
    public static final int shooterMotorOneID = 24;
    public static final int shooterMotorTwoID = 25;

    public static final int shooterSpeed = 60;

    public static final double velocityOffset = 5;
  }

  public static class uptakeConstants{
    public static final int uptakeMotorID = 4;

    public static final int uptakeSpeed = 80;
  }

  public static class spindexerConstants{
    public static final int spindexerMotorID = 5;

    public static final double spindexerSpeed = 100;
  }

  public static class intakeConstants{
    public static final int intakeMotorID = 3;
    public static final int intakeRotationMotorID = 8;
    public static final int intakeCANcoderID = 22;

    public static final double intakeSpeed = .7;
    public static final double intakeRotationSpeed = .4;

    public static final double intakeRatio = 250/21;//kraken rotations : intake rotations

    public static final Rotation2d intakeMax = Rotation2d.fromDegrees(90);
    public static final Rotation2d intakeMin = Rotation2d.fromDegrees(-68);

    public static final Rotation2d CANcoderOffset = Rotation2d.fromRotations(-0.179931640625);
  }

  public static class pigeonConstants{
    public static final int pigeonID = 1;
  }

    public static class VisionConstants{
        public static final double stdCoefficient = 0.1;

        public static final Pose2d objectDetectionRobotToCamera = new Pose2d(-0.254, 0, Rotation2d.fromDegrees(180));//12.5, 10, 180 degrees

        public static final Rotation2d intakeSideRelativeToFront = Rotation2d.fromDegrees(180);
    }

    public static class TurretConstants{
        public static final int TurretRotationID = 7;
        public static final int shooterElevationMotorID = 26;
        public static final int turretEncoderID = 27;

        public static final double encoderRatio = 30/12;

        public static final double turretRatio = 18.09912109375 - 3.68310546875;//30/12 * 150/26;//Kraken rotations : turret rotations, 14.345
        public static final double elevationRatio = 955/12;//Kraken rotations : turret rotations

        public static final Transform2d RobotToTurret = new Transform2d(-0.161925, -0.0889508, Rotation2d.fromDegrees(0));

        public static final Rotation2d TurretStartOffset = Rotation2d.fromDegrees(50);//relative to the cable chain (counterclockwise positive)
        public static final Rotation2d TurretCableChainPoint = Rotation2d.fromDegrees(590);//Relative to the front of the robot (clockwise positive)
        public static final Rotation2d TurretMin = Rotation2d.fromDegrees(90);
        public static final Rotation2d TurretMax = Rotation2d.fromDegrees(540);

        public static final double TurretVerticalOffset = 0.425;//meters
        public static final Rotation2d TurretMaxAngle = Rotation2d.fromDegrees(75);//from horizontal, old: 65
        public static final Rotation2d TurretMinAngle = Rotation2d.fromDegrees(50);//from horizontal, old: 40
        public static final Rotation2d TurretStartElevation = Rotation2d.fromDegrees(75);
    }

    public static class FieldConstants {
        public static final Translation3d HubFieldPoseBlue = new Translation3d(4.625,4.025, 1.8288);  
        public static final Translation3d HubFieldPoseRed = new Translation3d(11.925, 4.025, 1.8288);
        public static final Translation3d AimPose1 = new Translation3d(0, 0, 1.8288);
        public static final Translation3d AimPose2 = new Translation3d(1, 2, 0);
        public static final Translation3d AimPose3 = new Translation3d(4, 0, 0);

    }

    public static class SwerveConstants {
        // Drive and turn gear ratios for the mk 5 module
        public static final double RTurnRatio = 26.09;

        public static final double R1Drive = 7.03;
        public static final double R2Drive = 6.03;
        public static final double R3Drive = 5.27;

        public static final double mk4iDriveL2 = 6.75;
        public static final double mk4iRotate = 150/7;

        public static final PathConstraints telePathConstraints = new PathConstraints(1, 1, 2 * Math.PI, 4 * Math.PI);
        public static final double DrivekP = 0.02;
        public static final double DrivekI = 0.0;//0.1
        public static final double DrivekD = 0.00;
        public static final double DriveKS = 0.1;//for finding the kv and ks based off of each other -> 10 vel at 1.45 volt -> 1.45 - 0.3 = 1.15 / 10 = 0.115
        public static final double DriveKV = 0.0;//0.125

        public static final double TurnkP = 4;
        public static final double TurnkI = 0;
        public static final double TurnkD = 0;

        public static final double driveRamp = 0.2;

        public static final double maxSpeed = 5.12064;//meters per second
        public static final double maxAngularVelocity = 3.1154127;//radians per second

        public static final double HeadingOffset = 0;//degrees from forward
        public static final double gyroOffset = 90;

        public static final double wheelRadius = 2;
        public static final double wheelCircumference = (2 * Math.PI) * wheelRadius;
        public static final double wheelCircumferenceMeters = wheelCircumference * 0.0254;

        public static final int pigeonID = 1;

        //Track width and wheel base are measured in inches not meters 
        public static final double trackWidth = 23.35;
        public static final double wheelBase = 20.25;

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d[]{
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0)
            }
        );
    }

    //MODULE 1
    public static class FRConstants {
        public static final int driveMotorID = 13;
        public static final int turnMotorID = 14;
        public static final int CANCoderID = 15;

        public static final InvertedValue driveInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue turnInverted = InvertedValue.CounterClockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(355.95703125);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants FRConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }

    //MODULE 0
    public static class FLConstants {
        public static final int driveMotorID = 10;
        public static final int turnMotorID = 11;
        public static final int CANCoderID = 12;

        public static final InvertedValue driveInverted = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue turnInverted = InvertedValue.CounterClockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(176.1328125);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants FLConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }

    //MODULE 3
    public static class BRConstants {
        public static final int driveMotorID = 16;
        public static final int turnMotorID = 17;
        public static final int CANCoderID = 18;

        public static final InvertedValue driveInverted = InvertedValue.Clockwise_Positive;
        public static final InvertedValue turnInverted = InvertedValue.CounterClockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(174.990234375);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants BRConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }

    //MODULE 2
    public static class BLConstants {
        public static final int driveMotorID = 19;
        public static final int turnMotorID = 20;
        public static final int CANCoderID = 21;

        public static final InvertedValue driveInverted = InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue turnInverted = InvertedValue.CounterClockwise_Positive;

        public static final Rotation2d EncoderOffset = Rotation2d.fromDegrees(353.232421875);
        public static final SensorDirectionValue EncoderReversed = SensorDirectionValue.CounterClockwise_Positive;

        public static final SwerveModuleConstants BLConstants = new SwerveModuleConstants(driveMotorID, turnMotorID, CANCoderID, driveInverted, turnInverted, EncoderOffset, EncoderReversed);
    }

    public static class LEDConstants {
        public static final int LEDID = 1;
        public static final int LEDLength = 46;//79
        public static final int endgameLength = 46;

        public static final int LEDBrightness = 50;

        public static final int idleSpeed = 50;
        public static final int targetingSpeed = 1;
        public static final double activePeriodSoon = 0.1;
        public static final double activePeriod = 0.3;
    }
}
