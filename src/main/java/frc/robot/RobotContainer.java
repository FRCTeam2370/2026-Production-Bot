// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.intakeConstants;
import frc.robot.Commands.DriveOnX;
import frc.robot.Commands.FindADot;
import frc.robot.Commands.PIDToPoint;
import frc.robot.Commands.ResetGyro;
import frc.robot.Commands.RunSystemsCheck;
import frc.robot.Commands.SetDriveNeutralMode;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.ToggleDriveFeatures;
import frc.robot.Commands.ToggleTurretFeatures;
import frc.robot.Commands.XMode;
import frc.robot.Commands.Intake.DeployIntake;
import frc.robot.Commands.Intake.SetIntakePosAndSpeed;
import frc.robot.Commands.Shooter.ShootAtVelocity;
import frc.robot.Commands.Shooter.SimpleShootAtVelocity;
import frc.robot.Commands.TurretCommands.AimAtActiveAimPoint2;
import frc.robot.Commands.TurretCommands.PointTurretAndShootForTime;
import frc.robot.Commands.TurretCommands.PointTurretAndShootForTime2;
import frc.robot.Commands.TurretCommands.ZeroTurret;
import frc.robot.Subsystems.FieldInfo;
import frc.robot.Subsystems.FieldInfo.Dot;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ObjectDetection;
import frc.robot.Subsystems.OperatorTargetingSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.UptakeSubsystem;
import frc.robot.Subsystems.Vision;
import frc.robot.Utils.BallLogic;

public class RobotContainer {
  private ArrayList<Pose2d> dots = new ArrayList<Pose2d>(List.of(FieldConstants.dot1Pose, FieldConstants.dot2Pose, FieldConstants.dot3Pose));
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  public static final GenericHID dial = new GenericHID(2);

  public static boolean shouldDial = false;
  
  private final ObjectDetection mObjectDetection = new ObjectDetection();
  public final SwerveSubsystem mSwerve = new SwerveSubsystem(mObjectDetection);
  private final FieldInfo mFieldInfo = new FieldInfo();
  private final TurretSubsystem mTurretSubsystem = new TurretSubsystem(mSwerve);
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final SpindexerSubsystem mSpindexerSubsystem = new SpindexerSubsystem();
  private final UptakeSubsystem mUptakeSubsystem = new UptakeSubsystem();
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  private final Vision mVision = new Vision();
  private final LEDSubsystem mcLedSubsystem = new LEDSubsystem();
  //private final OperatorTargetingSubsystem mcOperatorTargetingSubsystem = new OperatorTargetingSubsystem();

  public final SendableChooser<Command> autoChooser;
  public final SendableChooser<Dot> dotChooser; 
  

  public RobotContainer() {    
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    //Put all NamedCommands here
    NamedCommands.registerCommand("Test", new ResetGyro(mSwerve));
    NamedCommands.registerCommand("Aim and Shoot For 3", new PointTurretAndShootForTime( 3, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("Aim and Shoot For 5", new PointTurretAndShootForTime( 5, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("Aim and Shoot For 9", new PointTurretAndShootForTime( 9, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("2 Aim and Shoot For 2", new PointTurretAndShootForTime2( 2, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("2 Aim and Shoot For 4", new PointTurretAndShootForTime2( 4, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("2 Aim and Shoot For 3", new PointTurretAndShootForTime2( 3, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("2 Aim and Shoot For 5", new PointTurretAndShootForTime2( 5, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("2 Aim and Shoot For 9", new PointTurretAndShootForTime2( 9, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("Beach for 3 Right", new PointTurretAndShootForTime( 3, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("Beach for 3 Left", new PointTurretAndShootForTime( 3, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("Deploy Intake", new DeployIntake(Rotation2d.fromDegrees(-67).getRotations(), 80, mIntakeSubsystem, mSwerve));
    NamedCommands.registerCommand("Aim and Shoot", new PointTurretAndShootForTime( 20, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("2 Aim and Shoot", new PointTurretAndShootForTime2( 20, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("Deploy Hintake", new DeployIntake(Rotation2d.fromDegrees(-40).getRotations(), 80, mIntakeSubsystem, mSwerve));
    NamedCommands.registerCommand("Prop Intake", new DeployIntake(Rotation2d.fromDegrees(30).getRotations(), 30, mIntakeSubsystem, mSwerve));
    NamedCommands.registerCommand("Feed Right", new PointTurretAndShootForTime( 2.5, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("Jiggle Intake", new SetIntakePosAndSpeed(Rotation2d.fromDegrees(-67).getRotations(), 60, mIntakeSubsystem, mSwerve));
    NamedCommands.registerCommand("Coast", new SetDriveNeutralMode(mSwerve, NeutralModeValue.Coast));
    NamedCommands.registerCommand("Dot Hop", new FindADot(mSwerve, ()-> (SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() > FieldConstants.dot1to2Y ? FieldConstants.dot1Pose : SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() > FieldConstants.dot2to3Y ? FieldConstants.dot2Pose : FieldConstants.dot3Pose), ()->true));
    NamedCommands.registerCommand("Stand Middle", new FindADot(mSwerve, ()-> FieldConstants.dot2Pose, ()-> false));
    NamedCommands.registerCommand("Stand Top", new FindADot(mSwerve, ()-> SwerveSubsystem.color.get() == Alliance.Blue ? FieldConstants.dot1Pose : FieldConstants.dot3Pose, ()-> false));
    NamedCommands.registerCommand("Stand Bottom", new FindADot(mSwerve, ()-> SwerveSubsystem.color.get() == Alliance.Blue ? FieldConstants.dot3Pose : FieldConstants.dot1Pose, ()-> false));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    dotChooser = new SendableChooser<>();
    dotChooser.setDefaultOption("NONE", Dot.NONE);
    dotChooser.addOption("DOT_HOP", Dot.DOT_HOP);
    dotChooser.addOption("STAND_GROUND", Dot.STAND_GROUND);
    SmartDashboard.putData("Dot Chooser", dotChooser);

    configureBindings();
  }

  //TODO: Add acceleration limit while shooting at the hub | change rotational velocity handling during shooting
  private void configureBindings() {
    mSwerve.setDefaultCommand(new TeleopSwerve(mSwerve, ()-> -driver.getRawAxis(0), ()-> driver.getRawAxis(1), ()-> driver.getRawAxis(4), ()-> false));
    mTurretSubsystem.setDefaultCommand(new AimAtActiveAimPoint2(mTurretSubsystem, mSwerve, ()-> SwerveSubsystem.shouldAutoTurret));
    mIntakeSubsystem.setDefaultCommand(new SetIntakePosAndSpeed(Rotation2d.fromDegrees(-67).getRotations(), 60, mIntakeSubsystem, mSwerve));

    driver.b().toggleOnFalse(new ToggleTurretFeatures());

    driver.x().toggleOnTrue(new XMode(mSwerve));

    driver.back().onTrue(new ResetGyro(mSwerve));

    //driver.y().toggleOnTrue(new SetElevationPos(TurretConstants.ElevationMinAngle.getDegrees(), mTurretSubsystem));

    driver.leftStick().toggleOnTrue(mSwerve.SweepAllianceWall());
    driver.povUp().whileTrue(new SimpleShootAtVelocity(mShooterSubsystem, -30));
    
    driver.rightTrigger().toggleOnTrue(new ShootAtVelocity(mShooterSubsystem, mUptakeSubsystem, mSpindexerSubsystem, mSwerve, mFieldInfo));
    driver.leftBumper().toggleOnTrue(new SetIntakePosAndSpeed(Rotation2d.fromDegrees(intakeConstants.intakeMax.getDegrees()).getRotations(), 0, mIntakeSubsystem, mSwerve));
    driver.povRight().toggleOnTrue(new SetIntakePosAndSpeed(Rotation2d.fromDegrees(-40).getRotations(), 60, mIntakeSubsystem, mSwerve));
    driver.povDown().whileTrue(mSwerve.PathfindToPose(()-> FieldConstants.dot1Pose));
    driver.rightStick().toggleOnTrue(new DriveOnX(mSwerve, ()-> -driver.getRawAxis(0)));

    driver.povLeft().whileTrue(new PIDToPoint(()-> (SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() > FieldConstants.dot1to2Y ? FieldConstants.dot1Pose : SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() > FieldConstants.dot2to3Y ? FieldConstants.dot2Pose : FieldConstants.dot3Pose), mSwerve, ()->false));
    //driver.povLeft().whileTrue(new FindADot(mSwerve, ()-> (SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() > FieldConstants.dot1to2Y ? FieldConstants.dot1Pose : SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() > FieldConstants.dot2to3Y ? FieldConstants.dot2Pose : FieldConstants.dot3Pose), ()-> true));
    //driver.leftTrigger().whileTrue(mSwerve.driveThroughBalls());
    //driver.povUp().whileTrue(mSwerve.driveToClosestBall(()-> mSwerve.getClosestBall()));
    //driver.povLeft().whileTrue(mSwerve.PathfindToPose(()-> FieldInfo.fieldPoints.ClimbLeft));
    //driver.leftTrigger().whileTrue(new IntakeControl(mIntakeSubsystem, -30));

    // operator.rightBumper().whileTrue(new ClimbForPercent(30, mClimberSubsystem));
    // operator.leftBumper().whileTrue(new ClimbForPercent(-30, mClimberSubsystem));
    // operator.a().onTrue(new SetClimberPos(0, mClimberSubsystem));
    // operator.b().onTrue(new SetClimberPos(230, mClimberSubsystem));
    // operator.x().onTrue(new SetClimberPos(115, mClimberSubsystem));

    // operator.y().onTrue(new EnableAirStrike(true));
    // operator.povDown().onTrue(new EnableAirStrike(false));

    operator.y().toggleOnTrue(new ToggleDriveFeatures());
    operator.b().toggleOnFalse(new ToggleTurretFeatures());
    operator.x().toggleOnTrue(new ZeroTurret());
    operator.leftBumper().onTrue(new SetDriveNeutralMode(mSwerve, NeutralModeValue.Brake));
    operator.rightBumper().onTrue(new SetDriveNeutralMode(mSwerve, NeutralModeValue.Coast));

    operator.povDown().toggleOnTrue(new RunSystemsCheck(mTurretSubsystem, mShooterSubsystem, mUptakeSubsystem, mSpindexerSubsystem));
    operator.povLeft().onTrue(Commands.runOnce(()-> TurretSubsystem.isShooting = true, mTurretSubsystem));
  }

  public Command getAutonomousCommand() {
    // if(dotChooser.getSelected() != null && dotChooser.getSelected() != Dot.NONE){
    //   return Commands.race(new WaitCommand(15), autoChooser.getSelected()).andThen(Commands.runOnce(()-> mSwerve.drive(new Translation2d(0,0), 0, true, false), mSwerve)).andThen(new FindADot(mSwerve, ()-> (SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() > FieldConstants.dot1to2Y ? FieldConstants.dot1Pose : SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() > FieldConstants.dot2to3Y ? FieldConstants.dot2Pose : FieldConstants.dot3Pose), ()-> (dotChooser.getSelected() == Dot.STAND_GROUND ? false : true)));
    // }else{
    //   return autoChooser.getSelected();
    // }
    return autoChooser.getSelected();
  }
}
