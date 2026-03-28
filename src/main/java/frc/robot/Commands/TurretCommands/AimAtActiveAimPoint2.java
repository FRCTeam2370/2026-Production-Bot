// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TurretCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAtActiveAimPoint2 extends Command {
  BooleanSupplier shouldTurret;
  SwerveSubsystem mSwerve;
  /** Creates a new AimAtActiveAimPoint2. */
  public AimAtActiveAimPoint2(TurretSubsystem mTurretSubsystem, SwerveSubsystem mSwerve, BooleanSupplier shouldTurret) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mSwerve = mSwerve;
    this.shouldTurret = shouldTurret;
    addRequirements(mTurretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(shouldTurret.getAsBoolean()){
    //   if(ShooterSubsystem.getVelocity() > 10 && TurretSubsystem.canElevate){
    //     TurretSubsystem.aimTurretAtPoint(new Pose2d(mSwerve.getTurretPointTowardsPoseJacobMethod(TurretSubsystem.activeAimPoint.aimPoint).aimPose.getX(), mSwerve.getTurretPointTowardsPoseJacobMethod(TurretSubsystem.activeAimPoint.aimPoint).aimPose.getY(), new Rotation2d()));
    //     TurretSubsystem.setElevation(mSwerve.getTurretPointTowardsPoseJacobMethod(TurretSubsystem.activeAimPoint.aimPoint).elevationAngleDegrees + 5);
    //   }else{
    //     TurretSubsystem.aimTurretAtPoint(new Pose2d(TurretSubsystem.activeAimPoint.aimPoint.getX(), TurretSubsystem.activeAimPoint.aimPoint.getY(), new Rotation2d()));
    //     TurretSubsystem.setElevation(TurretConstants.ElevationMaxAngle.getDegrees());
    //   }
    // }else{
    //   TurretSubsystem.aimTurretAtDegree(360);
    //   TurretSubsystem.setElevation(TurretConstants.ElevationMaxAngle.getDegrees());
    // }

      TurretSubsystem.aimTurretAtPoint(new Pose2d(mSwerve.getTurretPointTowardsPoseJacobMethod(TurretSubsystem.activeAimPoint.aimPoint).aimPose.getX(), mSwerve.getTurretPointTowardsPoseJacobMethod(TurretSubsystem.activeAimPoint.aimPoint).aimPose.getY(), new Rotation2d()), false);
      
      if(TurretSubsystem.canElevate){
        TurretSubsystem.setElevation(mSwerve.getTurretPointTowardsPoseJacobMethod(TurretSubsystem.activeAimPoint.aimPoint).elevationAngleDegrees + 5);
      }else{
        TurretSubsystem.setElevation(TurretConstants.ElevationMaxAngle.getDegrees());
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TurretSubsystem.aimTurretAtDegree(360);
    TurretSubsystem.setElevation(TurretConstants.ElevationMaxAngle.getDegrees());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
