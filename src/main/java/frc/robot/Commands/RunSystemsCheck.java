// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.spindexerConstants;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.SpindexerSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.Subsystems.UptakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunSystemsCheck extends Command {
  /** Creates a new RunSystemsCheck. */
  public RunSystemsCheck(TurretSubsystem mTurretSubsystem, ShooterSubsystem mShooterSubsystem, UptakeSubsystem mUptakeSubsystem, SpindexerSubsystem mSpindexerSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mShooterSubsystem, mSpindexerSubsystem, mTurretSubsystem, mUptakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    TurretSubsystem.setElevation(TurretConstants.ElevationMaxAngle.getDegrees());
    TurretSubsystem.aimTurretAtDegree(398.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ShooterSubsystem.shootWithVelocity(15);
    UptakeSubsystem.uptakeWithVelocity(50);
    SpindexerSubsystem.spindexrWithVelocity(50);
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
