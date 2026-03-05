// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ClimberCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbForPercent extends Command {
  double percent0to100;
  /** Creates a new ClimbForPercent. */
  public ClimbForPercent(double percent0to100, ClimberSubsystem mClimberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.percent0to100 = percent0to100;
    addRequirements(mClimberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ClimberSubsystem.runClimberAtSpeed(percent0to100);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ClimberSubsystem.runClimberAtSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
