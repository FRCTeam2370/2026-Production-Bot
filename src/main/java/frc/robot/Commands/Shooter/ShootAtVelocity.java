// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Red;
import frc.robot.Constants.spindexerConstants;
import frc.robot.Constants.uptakeConstants;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.UptakeSubsystem;
import frc.robot.Utils.TurretLogic.TurretAimPose;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.SwerveSubsystem;
import frc.robot.Subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootAtVelocity extends Command {
  double vel;
  boolean usingLower;
  SwerveSubsystem mSwerve;
  /** Creates a new ShootAtVeolcity. */
  public ShootAtVelocity(ShooterSubsystem mShooterSubsystem, UptakeSubsystem mUptakeSubsystem, SpindexerSubsystem mSpindexerSubsystem, SwerveSubsystem mSwerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mSwerve = mSwerve;
    addRequirements(mShooterSubsystem, mUptakeSubsystem, mSpindexerSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    TurretAimPose aimpose = mSwerve.getTurretPointTowardsPose(Red.HubFieldPoseRed);
    vel = aimpose.vel;
    usingLower = aimpose.usingLower;
    ShooterSubsystem.shootWithVelocity(vel);
    if(ShooterSubsystem.getVelocity() > vel * 0.9 && TurretSubsystem.canShoot){
      UptakeSubsystem.uptakeWithVelocity(uptakeConstants.uptakeSpeed);
      SpindexerSubsystem.spindexrWithVelocity(spindexerConstants.spindexerSpeed);
    }else{
      UptakeSubsystem.uptakeWithVelocity(-20);
      SpindexerSubsystem.spindexrWithVelocity(0);
    }
    SmartDashboard.putBoolean("using Lower", usingLower);
    SmartDashboard.putNumber("Expected Shooter vel", vel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ShooterSubsystem.shootWithVelocity(0);
    UptakeSubsystem.uptakeWithVelocity(0);
    SpindexerSubsystem.spindexrWithVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}