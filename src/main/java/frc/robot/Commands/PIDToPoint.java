// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.lang.invoke.ConstantBootstraps;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

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
  Supplier<Pose2d> pointah;
  BooleanSupplier searchMore;
  /** Creates a new PIDToPoint. */
  public PIDToPoint(Supplier<Pose2d> pointah, SwerveSubsystem mSwerve, BooleanSupplier searchMore) {
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
    mSwerve.drive(new Translation2d(Math.max(-1, Math.min(SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() - pointah.get().getY(), 1)),
          -Math.max(-1, Math.min(SwerveSubsystem.poseEstimator.getEstimatedPosition().getX() - pointah.get().getX(), 1))).times(2/*SwerveConstants.maxSpeed*/), 
          SwerveSubsystem.poseEstimator.getEstimatedPosition().getRotation().getRotations() - pointah.get().getRotation().getRotations(), 
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
    if(searchMore.getAsBoolean() && timer.get() > 0.5 && Math.abs(SwerveSubsystem.poseEstimator.getEstimatedPosition().getX() - pointah.get().getX()) > 0.05 && Math.abs(SwerveSubsystem.poseEstimator.getEstimatedPosition().getY() - pointah.get().getY()) > 0.05){
      return true;
    }else{
      return false;
    }
  }
}
