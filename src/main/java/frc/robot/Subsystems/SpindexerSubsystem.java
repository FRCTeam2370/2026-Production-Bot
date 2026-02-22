// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.spindexerConstants;

public class SpindexerSubsystem extends SubsystemBase {
  public static TalonFX spindexerMotor = new TalonFX(spindexerConstants.spindexerMotorID, "*");

  public static TalonFXConfiguration spindexerMotorConfig = new TalonFXConfiguration();

  public static VelocityDutyCycle spindexerVelocityDutyCycle = new VelocityDutyCycle(0);

  /** Creates a new SpindexerSubsystem. */
  public SpindexerSubsystem() {
    spindexerConfiguration();
  }

  public static void spindexrWithVelocity(double speed) {
    if (speed != 0) {
      spindexerMotor.setControl(spindexerVelocityDutyCycle.withVelocity(speed));
    } else {
      spindexerMotor.set(speed);
    }
  }

  public static void spindexerWithoutVelocity(double speed) {
    spindexerMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Spindexer  Motor Current", spindexerMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Spindexer Velocity", spindexerMotor.getVelocity().getValueAsDouble());
  }

  public static void spindexerConfiguration() {
    spindexerMotor.setNeutralMode(NeutralModeValue.Coast);

    spindexerMotorConfig.CurrentLimits.StatorCurrentLimit = 40;

    spindexerMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; 
    // spindexerMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;

    spindexerMotorConfig.Slot0.kV = 0.01;
    spindexerMotorConfig.Slot0.kP = 0.01;
    spindexerMotorConfig.Slot0.kI = 0;
    spindexerMotorConfig.Slot0.kD = 0;

    spindexerMotor.getConfigurator().apply(spindexerMotorConfig);
  }
}