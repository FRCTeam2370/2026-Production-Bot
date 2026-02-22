// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  public static TalonFX shooterMotorOne = new TalonFX(shooterConstants.shooterMotorOneID, "*");
  public static TalonFX shooterMotorTwo = new TalonFX(shooterConstants.shooterMotorTwoID, "*");

  public static TalonFXConfiguration shooterMotorOneConfig = new TalonFXConfiguration();
  public static TalonFXConfiguration shooterMotorTwoConfig = new TalonFXConfiguration();

  public static VelocityDutyCycle shooterVelocityDutyCycle = new VelocityDutyCycle(0);

  public ShooterSubsystem() {
    shooterMotorConfig();
  }

  public static void shootWithVelocity(double speed) {
    if (speed != 0) {
      shooterMotorOne.setControl(shooterVelocityDutyCycle.withVelocity(speed));
    } else {
      shooterMotorOne.set(speed);
    }
  }

  public static double getVelocity(){
    return shooterMotorOne.getVelocity().getValueAsDouble();
  }

  public static void shootPrototype(double speed) {
    shooterMotorOne.set(speed);
    shooterMotorTwo.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Motor One Velocity", shooterMotorOne.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Motor Two Velocity", shooterMotorTwo.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Motor One Current", shooterMotorOne.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Motor Two Current", shooterMotorTwo.getStatorCurrent().getValueAsDouble());
  }

  public static void shooterMotorConfig() {
    shooterMotorOne.setNeutralMode(NeutralModeValue.Coast);
    shooterMotorTwo.setNeutralMode(NeutralModeValue.Coast);

    shooterMotorOneConfig.CurrentLimits.StatorCurrentLimit = 40;
    shooterMotorTwoConfig.CurrentLimits.StatorCurrentLimit = 40;

    shooterMotorOneConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterMotorTwoConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    shooterMotorOneConfig.Slot0.kV = 0.01;
    shooterMotorTwoConfig.Slot0.kV = 0.01;

    shooterMotorOneConfig.Slot0.kP = 0.095;
    shooterMotorTwoConfig.Slot0.kP = 0.095;
    shooterMotorOneConfig.Slot0.kI = 0;
    shooterMotorTwoConfig.Slot0.kI = 0;
    shooterMotorOneConfig.Slot0.kD = 0;
    shooterMotorTwoConfig.Slot0.kD = 0;

    shooterMotorOne.getConfigurator().apply(shooterMotorOneConfig);
    shooterMotorTwo.getConfigurator().apply(shooterMotorTwoConfig);

    shooterMotorTwo.setControl(new Follower(shooterMotorOne.getDeviceID(), MotorAlignmentValue.Opposed));
  }
}
