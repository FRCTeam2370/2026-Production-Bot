// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  public static TalonFX intakeMotor = new TalonFX(intakeConstants.intakeMotorID, "*");
  public static TalonFX intakeRotationMotor = new TalonFX(intakeConstants.intakeRotationMotorID, "*");
  public static CANcoder intakeCANcoder = new CANcoder(intakeConstants.intakeCANcoderID);

  public static TalonFXConfiguration intakeMotorConfig = new TalonFXConfiguration();
  public static TalonFXConfiguration intakeRotationMotorConfig = new TalonFXConfiguration();
  private static CANcoderConfiguration CANcoderConfig = new CANcoderConfiguration();

  public static VelocityDutyCycle intakVelocityDutyCycle = new VelocityDutyCycle(0);
  public static MotionMagicDutyCycle intakeRotatoinDutyCycle = new MotionMagicDutyCycle(0);
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
    SmartDashboard.putNumber("intake position current", intakeRotationMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake CANcoder val", intakeCANcoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Degrees", Rotation2d.fromRotations(KrakenToIntake(intakeRotationMotor.getPosition().getValueAsDouble())).getDegrees());
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

  public static void setIntakePoseVel(double vel){
    intakeRotationMotor.setControl(intakePosVelCycle.withVelocity(vel));
  }

  public static void setIntakePos(double pos){
    intakeRotationMotor.setControl(intakeRotatoinDutyCycle.withPosition(intakeToKraken(pos)));
  }

  public static void intakeConfig() {
    intakeMotor.setNeutralMode(NeutralModeValue.Brake);

    intakeMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    intakeMotorConfig.Slot0.kP = 0.01;
    intakeMotorConfig.Slot0.kV = 0.01;

    intakeMotor.getConfigurator().apply(intakeMotorConfig);
  }

  public static void intakeRotationConfig() {
    intakeRotationMotor.setPosition(intakeToKraken(intakeCANcoder.getAbsolutePosition().getValueAsDouble() - intakeConstants.CANcoderOffset.getRotations() + intakeConstants.intakeMax.getRotations()));

    intakeRotationMotor.setNeutralMode(NeutralModeValue.Coast);

    intakeRotationMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    intakeRotationMotorConfig.Feedback.RotorToSensorRatio = intakeConstants.intakeRatio;
    CANcoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

    intakeRotationMotorConfig.Slot0.kP = 0.1;
    intakeRotationMotorConfig.Slot0.kI = 0;
    intakeRotationMotorConfig.Slot0.kD = 0.0;

    intakeRotationMotorConfig.Slot0.kG = 0.03;

    intakeRotationMotorConfig.MotionMagic.MotionMagicAcceleration = 100;
    intakeRotationMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 40;

    intakeRotationMotor.getConfigurator().apply(intakeRotationMotorConfig);
    intakeCANcoder.getConfigurator().apply(CANcoderConfig);
  }

  private static double intakeToKraken(double rot){
    return rot * intakeConstants.intakeRatio;
  }

  private static double KrakenToIntake(double krak){
    return krak / intakeConstants.intakeRatio;
  }
}
