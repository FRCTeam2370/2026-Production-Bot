// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FindIntakeKG extends Command {
  /** Creates a new FindIntakeKG. */
  double vel = 0;
  public FindIntakeKG(IntakeSubsystem mIntakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mIntakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(RobotContainer.driver.a().getAsBoolean() == true){
      vel += 0.01;
    }else if(RobotContainer.driver.b().getAsBoolean() == true){
      vel -= 0.01;
    }

    IntakeSubsystem.setIntakePoseVel(vel);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    IntakeSubsystem.intakeRotationMotor.set(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
