// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.LEDSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.LEDSubsystem.LEDState;
import frc.robot.Subsystems.TurretSubsystem.ActiveAimPose;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopSwerve extends Command {
  private SwerveSubsystem mSwerve;
  private DoubleSupplier xSup, ySup, rotSup;
  private BooleanSupplier robotCentricSup;
  private SlewRateLimiter xLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(3);
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  /** Creates a new TeleopSwerve. */
  public TeleopSwerve(SwerveSubsystem mSwerve, DoubleSupplier xSup, DoubleSupplier ySup, DoubleSupplier rotSup, BooleanSupplier robotCentricSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mSwerve = mSwerve;
    addRequirements(mSwerve);
    this.xSup = xSup;
    this.ySup = ySup;
    this.rotSup = rotSup;
    this.robotCentricSup = robotCentricSup;
    SwerveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xVal = Math.abs(xSup.getAsDouble()) < 0.05 ? 0 : xSup.getAsDouble();
    double yVal = Math.abs(ySup.getAsDouble()) < 0.05 ? 0 : ySup.getAsDouble();
    double rotVal = Math.abs(rotSup.getAsDouble()) < 0.05 ? 0 : rotSup.getAsDouble();

    if(ShooterSubsystem.getVelocity() > 5 && LEDSubsystem.mLEDState == LEDState.Hub){
      ActiveAimPose pose = TurretSubsystem.activeAimPoint;
      double xDistanceToTarget = pose.aimPoint.getX() - SwerveSubsystem.poseEstimator.getEstimatedPosition().getX();
      double yDistanceToTarget = pose.aimPoint.getY() - SwerveSubsystem.poseEstimator.getEstimatedPosition().getY();
      double totalDistanceToTarget = Math.sqrt(Math.pow(xDistanceToTarget, 2) + Math.pow(yDistanceToTarget, 2));
      double xVelocity = yVal * Constants.SwerveConstants.maxSpeed;
      double yVelocity = xVal * Constants.SwerveConstants.maxSpeed;

      if((xVelocity * xDistanceToTarget) > 0){
        yVal /= Math.abs(totalDistanceToTarget/yDistanceToTarget);
      }
      if((yVelocity * yDistanceToTarget) < 0){
        xVal /= Math.abs(totalDistanceToTarget/xDistanceToTarget);
      }
    }

    xVal = SwerveSubsystem.Clamp(xVal - SwerveSubsystem.getTrenchOffsetY(), -1, 1);

    xVal *= SwerveSubsystem.color.isPresent() && SwerveSubsystem.color.get() == Alliance.Blue ? -1 : 1;
    yVal *= SwerveSubsystem.color.isPresent() && SwerveSubsystem.color.get() == Alliance.Blue ? -1 : 1;

    mSwerve.drive(new Translation2d(xLimiter.calculate(xVal), yLimiter.calculate(yVal)).times(SwerveConstants.maxSpeed), rotLimiter.calculate(rotVal * 0.2), !robotCentricSup.getAsBoolean(), true);
  }
}
