// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.lang.invoke.ConstantBootstraps;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PIDToPoint extends Command {
  PIDController dotPID = new PIDController(1, 0, 0.01);
  Timer timer = new Timer();
  SwerveSubsystem mSwerve;
  Pose2d pointah;
  boolean searchMore = false;
  /** Creates a new PIDToPoint. */
  public PIDToPoint(Pose2d pointah, SwerveSubsystem mSwerve, boolean searchMore) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.searchMore = searchMore;
    this.mSwerve = mSwerve;
    this.pointah = pointah;
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    mSwerve.setDriveCoast(NeutralModeValue.Brake);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mSwerve.drive(new Translation2d(-Math.max(-1, Math.min(SwerveSubsystem.poseEstimator.getEstimatedPosition().getX() - pointah.getX(), 1)),
          Math.max(-1, Math.min(SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() - pointah.getY(), 1))).times(1/*SwerveConstants.maxSpeed*/), 
          SwerveSubsystem.poseEstimator.getEstimatedPosition().getRotation().getRotations() - pointah.getRotation().getRotations(), 
          true,
          false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(searchMore && timer.get() > 1){
      return true;
    }else{
      return false;
    }
  }
}
