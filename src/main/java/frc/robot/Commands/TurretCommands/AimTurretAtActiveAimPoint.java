// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TurretCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.LEDSubsystem.LEDState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimTurretAtActiveAimPoint extends Command {
  private SwerveSubsystem mSwerve;
  private LEDState lastState;
  /** Creates a new AimTurretAtActiveAimPoint. */
  public AimTurretAtActiveAimPoint(SwerveSubsystem mSwerve, TurretSubsystem mTurretSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mSwerve = mSwerve;
    addRequirements(mTurretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastState = TurretSubsystem.activeAimPoint.ledState;
    LEDSubsystem.mLEDState = lastState;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ShooterSubsystem.getVelocity() > 10){
      TurretSubsystem.aimTurretAtPoint(new Pose2d(mSwerve.getTurretPointTowardsPose(TurretSubsystem.activeAimPoint.aimPoint).aimPose.getX(), mSwerve.getTurretPointTowardsPose(TurretSubsystem.activeAimPoint.aimPoint).aimPose.getY(), new Rotation2d()));
      TurretSubsystem.setElevation(mSwerve.getTurretPointTowardsPose(TurretSubsystem.activeAimPoint.aimPoint).elevationAngleDegrees + 5);
    }else{
      TurretSubsystem.aimTurretAtPoint(new Pose2d(TurretSubsystem.activeAimPoint.aimPoint.getX(), TurretSubsystem.activeAimPoint.aimPoint.getY(), new Rotation2d()));
      TurretSubsystem.setElevation(TurretConstants.TurretMaxAngle.getDegrees());
    }

    if(lastState != TurretSubsystem.activeAimPoint.ledState){
      lastState = TurretSubsystem.activeAimPoint.ledState;
      LEDSubsystem.mLEDState = lastState;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TurretSubsystem.aimTurretAtDegree(540);
    TurretSubsystem.setElevation(TurretConstants.TurretMaxAngle.getDegrees());
    LEDSubsystem.mLEDState = LEDState.Off;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
