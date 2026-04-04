// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TurretCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.LEDSubsystem.LEDState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PointTurretAtPointForTime2 extends Command {
  private SwerveSubsystem mSwerve;
  private double seconds;
  private Timer timer = new Timer();
  /** Creates a new PointTurretAtPoint. */
  public PointTurretAtPointForTime2(double seconds, TurretSubsystem mTurretSubsystem, SwerveSubsystem mSwerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mSwerve = mSwerve;
    this.seconds = seconds;
    addRequirements(mTurretSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    LEDSubsystem.mLEDState = LEDState.Hub;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TurretSubsystem.aimTurretAtPoint(new Pose2d(mSwerve.getTurretPointTowardsPoseJacobMethod(TurretSubsystem.activeAimPoint.aimPoint).aimPose.getX(), mSwerve.getTurretPointTowardsPoseJacobMethod(TurretSubsystem.activeAimPoint.aimPoint).aimPose.getY(), new Rotation2d()), false);
    if(TurretSubsystem.canElevate){
      TurretSubsystem.setElevation(mSwerve.getTurretPointTowardsPoseJacobMethod(TurretSubsystem.activeAimPoint.aimPoint).elevationAngleDegrees);
    }else{
      TurretSubsystem.setElevation(TurretConstants.ElevationMaxAngle.getDegrees());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TurretSubsystem.aimTurretAtDegree(360);
    TurretSubsystem.setElevation(TurretConstants.ElevationMaxAngle.getDegrees());
    LEDSubsystem.mLEDState = LEDState.Off;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() >= seconds){
      timer.stop();
      return true;
    }else{
      return false;
    }
  }
}
