// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  public static TalonFX ClimberMotor = new TalonFX(ClimberConstants.ClimberID, "*");
  public static TalonFXConfiguration ClimberConfig = new TalonFXConfiguration();

  private static PositionDutyCycle ClimberPosCycle = new PositionDutyCycle(0);
  private static MotionMagicDutyCycle ClimberMagicCycle = new MotionMagicDutyCycle(0);
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    configClimber();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberMotor Pos", ClimberMotor.getPosition().getValueAsDouble());
  }

  public static void runClimberAtSpeed(double percent0to100){
    percent0to100 /= 100;
    ClimberMotor.set(percent0to100);
  }

  public static void setClimberPos(double pos){
    ClimberMotor.setControl(ClimberMagicCycle.withPosition(pos));
  }

  private static void configClimber(){
    ClimberMotor.setNeutralMode(NeutralModeValue.Brake);

    ClimberConfig.Slot0.kP = 0.05;
    ClimberConfig.Slot0.kI = 0.0;
    ClimberConfig.Slot0.kD = 0.0;

    ClimberConfig.MotionMagic.MotionMagicAcceleration = 240;
    ClimberConfig.MotionMagic.MotionMagicCruiseVelocity = 160;

    ClimberMotor.setPosition(230);
    ClimberConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    ClimberMotor.getConfigurator().apply(ClimberConfig);
  }
}
