// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  public static TalonFX intakeMotor = new TalonFX(intakeConstants.intakeMotorID, "*");
  public static TalonFX intakeRotationMotor = new TalonFX(intakeConstants.intakeRotationMotorID, "*");

  public static TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
  public static TalonFXConfiguration intakeRotationMotorConfig = new TalonFXConfiguration();

  public static VelocityDutyCycle intakVelocityDutyCycle = new VelocityDutyCycle(0);
  public static PositionDutyCycle intakeRotatoinDutyCycle = new PositionDutyCycle(0);
  private static VelocityDutyCycle intakePosVelCycle = new VelocityDutyCycle(0);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeConfig();
    intakeRotationConfig();
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Current", intakeMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake Pos", intakeRotationMotor.getPosition().getValueAsDouble());
  }

  public static void intakeWithVelocity(double speed) {
    if (speed != 0) {
      intakeMotor.setControl(intakVelocityDutyCycle.withVelocity(speed));
    } else {
      intakeMotor.set(speed);
    }
  }

  public static void intakeWithoutVelocity(double speed) {
    intakeMotor.set(speed);
  }

  public static void rotateIntake(double speed) {
    intakeRotationMotor.set(speed);
  }

  public static void intakeSetPoint(double position) {
    intakeRotationMotor.setControl(intakeRotatoinDutyCycle.withPosition(position));
  }

  public static void setIntakePoseVel(double vel){
    intakeRotationMotor.setControl(intakePosVelCycle.withVelocity(vel));
  }

  public static void intakeConfig() {
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakeMotorConfig.Slot0.kP = 0.01;
    intakeMotorConfig.Slot0.kV = 0.01;

    intakeMotor.getConfigurator().apply(intakeMotorConfig);
  }

  public static void intakeRotationConfig() {
    intakeRotationMotor.setPosition(0);

    intakeRotationMotor.setNeutralMode(NeutralModeValue.Coast);

    intakeRotationMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    intakeRotationMotorConfig.Slot0.kP = 0.05;
    intakeRotationMotorConfig.Slot0.kI = 0;
    intakeRotationMotorConfig.Slot0.kD = 0;

    intakeRotationMotorConfig.Slot0.kG = 0.0;

    intakeRotationMotor.getConfigurator().apply(intakeRotationMotorConfig);
  }
}
