// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Commands.DriveOnX;
import frc.robot.Commands.ResetGyro;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.ClimberCommands.ClimbForPercent;
import frc.robot.Commands.ClimberCommands.SetClimberPos;
import frc.robot.Commands.Intake.DeployIntake;
import frc.robot.Commands.Intake.SetIntakePosAndSpeed;
import frc.robot.Commands.Shooter.ShootAtVelocity;
import frc.robot.Commands.TurretCommands.AimTurretAtActiveAimPoint;
import frc.robot.Commands.TurretCommands.EnableAirStrike;
import frc.robot.Commands.TurretCommands.PointTurretAndShootForTime;
import frc.robot.Commands.TurretCommands.SetElevationPos;
import frc.robot.Constants.FieldConstants.Red;
import frc.robot.Subsystems.ClimberSubsystem;
import frc.robot.Subsystems.FieldInfo;
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

public class RobotContainer {
  public static final CommandXboxController driver = new CommandXboxController(0);
  public static final CommandXboxController operator = new CommandXboxController(1);
  public static final GenericHID dial = new GenericHID(2);

  public static boolean shouldDial = false;
  
  private final ObjectDetection mObjectDetection = new ObjectDetection();
  private final SwerveSubsystem mSwerve = new SwerveSubsystem(mObjectDetection);
  private final FieldInfo mFieldInfo = new FieldInfo();
  private final TurretSubsystem mTurretSubsystem = new TurretSubsystem();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final SpindexerSubsystem mSpindexerSubsystem = new SpindexerSubsystem();
  private final UptakeSubsystem mUptakeSubsystem = new UptakeSubsystem();
  private final ShooterSubsystem mShooterSubsystem = new ShooterSubsystem();
  private final Vision mVision = new Vision();
  private final LEDSubsystem mLedSubsystem = new LEDSubsystem();
  private final ClimberSubsystem mClimberSubsystem = new ClimberSubsystem();
  private final OperatorTargetingSubsystem mOperatorTargetingSubsystem = new OperatorTargetingSubsystem();

  private final SendableChooser<Command> autoChooser;
  

  public RobotContainer() {    
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

    //Put all NamedCommands here
    NamedCommands.registerCommand("Test", new ResetGyro(mSwerve));
    NamedCommands.registerCommand("Aim and Shoot For 3", new PointTurretAndShootForTime(TurretSubsystem.activeAimPoint.aimPoint, 3, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("Aim and Shoot For 5", new PointTurretAndShootForTime(TurretSubsystem.activeAimPoint.aimPoint, 5, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("Deploy Intake", new DeployIntake(Rotation2d.fromDegrees(-67).getRotations(), 80, mIntakeSubsystem, mSwerve));
    NamedCommands.registerCommand("Aim and Shoot", new PointTurretAndShootForTime(TurretSubsystem.activeAimPoint.aimPoint, 20, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));
    NamedCommands.registerCommand("Deploy Hintake", new DeployIntake(Rotation2d.fromDegrees(-40).getRotations(), 80, mIntakeSubsystem, mSwerve));
    NamedCommands.registerCommand("Prop Intake", new DeployIntake(Rotation2d.fromDegrees(45).getRotations(), 30, mIntakeSubsystem, mSwerve));
    NamedCommands.registerCommand("Feed Right", new PointTurretAndShootForTime(FieldInfo.fieldPoints.PassPose2, 2.5, mTurretSubsystem, mSwerve, mUptakeSubsystem, mSpindexerSubsystem, mShooterSubsystem));

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    
    configureBindings();
  }

  //TODO: 35 degrees from the wall is best for picking up against the wall
  private void configureBindings() {
    mSwerve.setDefaultCommand(new TeleopSwerve(mSwerve, ()-> -driver.getRawAxis(0), ()-> driver.getRawAxis(1), ()-> driver.getRawAxis(4), ()-> false, true));
    driver.b().toggleOnTrue(new AimTurretAtActiveAimPoint(mSwerve, mTurretSubsystem));

    driver.back().onTrue(new ResetGyro(mSwerve));

    driver.rightTrigger().toggleOnTrue(new ShootAtVelocity(mShooterSubsystem, mUptakeSubsystem, mSpindexerSubsystem, mSwerve));
    driver.leftBumper().toggleOnTrue(new SetIntakePosAndSpeed(Rotation2d.fromDegrees(-67).getRotations(), 60, mIntakeSubsystem, mSwerve));
    driver.povRight().toggleOnTrue(new SetIntakePosAndSpeed(Rotation2d.fromDegrees(-40).getRotations(), 60, mIntakeSubsystem, mSwerve));
    //driver.b().toggleOnTrue(new PointTurretAtPoint(FieldConstants.HubFieldPoseRed, mTurretSubsystem, mSwerve));
    //driver.x().onTrue(new SetTurretRotation(Rotation2d.fromDegrees(360).getRotations(), mTurretSubsystem));
    //driver.y().toggleOnTrue(new PointTurretAtPoint(FieldConstants.AimPose1, mTurretSubsystem, mSwerve));
    // driver.x().toggleOnTrue(new PointTurretAtPoint(FieldConstants.AimPose2, mTurretSubsystem, mSwerve));

    driver.a().toggleOnTrue(new DriveOnX(mSwerve, ()-> -driver.getRawAxis(0)));
    driver.leftTrigger().whileTrue(mSwerve.driveThroughBalls());
    driver.povUp().whileTrue(mSwerve.driveToClosestBall(()-> mSwerve.getClosestBall()));
    driver.povLeft().whileTrue(mSwerve.PathfindToPose(()-> FieldInfo.fieldPoints.ClimbRight));

    operator.rightBumper().whileTrue(new ClimbForPercent(30, mClimberSubsystem));
    operator.leftBumper().whileTrue(new ClimbForPercent(-30, mClimberSubsystem));
    operator.a().onTrue(new SetClimberPos(0, mClimberSubsystem));
    operator.b().onTrue(new SetClimberPos(230, mClimberSubsystem));
    operator.x().onTrue(new SetClimberPos(115, mClimberSubsystem));

    operator.y().onTrue(new EnableAirStrike(true));
    operator.povDown().onTrue(new EnableAirStrike(false));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
