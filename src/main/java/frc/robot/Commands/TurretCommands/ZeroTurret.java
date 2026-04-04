// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.TurretCommands;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.TurretSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZeroTurret extends Command {
  Boolean isDone = false;
  /** Creates a new ZeroTurret. */
  public ZeroTurret() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.operator.leftTrigger().getAsBoolean()){
      TurretSubsystem.aimTurretAtDegree(TurretSubsystem.getTurretRotation().getDegrees() - 1);
    }else if(RobotContainer.operator.rightTrigger().getAsBoolean()){
      TurretSubsystem.aimTurretAtDegree(TurretSubsystem.getTurretRotation().getDegrees() + 1);
    }

    if(RobotContainer.operator.a().getAsBoolean()){
      TurretSubsystem.resetTurret();
      isDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
