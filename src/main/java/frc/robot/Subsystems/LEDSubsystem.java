// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */

  //LED Setup Settings
  public static AddressableLED TurretLED = new AddressableLED(LEDConstants.LEDID);  
  public static AddressableLEDBuffer turretLedBuffer = new AddressableLEDBuffer(LEDConstants.LEDLength);
  public static AddressableLEDBuffer endgameBuffer = new AddressableLEDBuffer(LEDConstants.endgameLength);

  //LED Colors
  public static Color idleLeading = new Color("#97d700");
  public static Color hubTarget = new Color("#80ff00");
  public static Color pointTarget = new Color("#ff8000");
  
  //LED Animation Parameters
  Distance ledSpacing = Meter.of(0.3048/19);
  private final static Timer endgameTimer = new Timer();

    //Off Animation
    LEDPattern offPattern = LEDPattern.kOff;

    //Idle Animation
    Map<Double, Color> idleMaskSteps = Map.of(0.0, Color.kWhite, .1, Color.kBlack);
    
    LEDPattern idleBase = LEDPattern.solid(idleLeading);
    LEDPattern idleMask = LEDPattern.steps(idleMaskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(LEDConstants.idleSpeed));
    LEDPattern idlePattern = idleBase.mask(idleMask).atBrightness(Percent.of(LEDConstants.LEDBrightness));
  
    //Hub Targeting Animation
    LEDPattern hubBase = LEDPattern.solid(hubTarget);
    LEDPattern hubPattern = hubBase.breathe(Second.of(LEDConstants.targetingSpeed)).atBrightness(Percent.of(LEDConstants.LEDBrightness));;
  
    //Point Targeting Animation
    LEDPattern pointBase = LEDPattern.solid(pointTarget);
    LEDPattern pointPattern = pointBase.breathe(Second.of(LEDConstants.targetingSpeed)).atBrightness(Percent.of(LEDConstants.LEDBrightness));;

    //Red Active Animation
    LEDPattern redPrepareBase = LEDPattern.solid(Color.kRed);
    LEDPattern redPreparePattern = redPrepareBase.blink(Second.of(LEDConstants.activePeriodSoon));
  
    //Blue Active Animation
    LEDPattern bluePrepareBase = LEDPattern.solid(Color.kBlue);
    LEDPattern bluePreparePattern = bluePrepareBase.blink(Second.of(LEDConstants.activePeriodSoon));
  
    //Red Active Animation
    LEDPattern redBase = LEDPattern.solid(Color.kRed);
    LEDPattern redPattern = redBase.blink(Second.of(LEDConstants.activePeriod)).atBrightness(Percent.of(LEDConstants.LEDBrightness));;
  
    //Blue Active Animation
    LEDPattern blueBase = LEDPattern.solid(Color.kBlue);
    LEDPattern bluePattern = blueBase.blink(Second.of(LEDConstants.activePeriod)).atBrightness(Percent.of(LEDConstants.LEDBrightness));
  
    //Add .reversed() after endgameTimer.get() / 30.0)) if LED strip is backwards
    //Endgame Red Animation
    LEDPattern redEndPattern = LEDPattern.solid(Color.kRed).mask(LEDPattern.progressMaskLayer(() -> 1.0 - (endgameTimer.get() / 30.0))).atBrightness(Percent.of(LEDConstants.LEDBrightness));;
    
    //Endgame Blue Animation
    LEDPattern blueEndPattern = LEDPattern.solid(Color.kBlue).mask(LEDPattern.progressMaskLayer(() -> 1.0 - (endgameTimer.get() / 30.0))).atBrightness(Percent.of(LEDConstants.LEDBrightness));;
  
    public static enum LEDState {
      Idle,
      Hub,
      Point,
      Red,
      Blue,
      EndgameRed,
      EndgameBlue,
      PrepareRed,
      PrepareBlue,
      Off
    }
  
    public static LEDState mLEDState = LEDState.Off;
  
    public LEDSubsystem() {
      TurretLED.setLength(turretLedBuffer.getLength());
  
      TurretLED.setData(turretLedBuffer);
      TurretLED.start();
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      switch (mLEDState) {
        case Idle:
          idlePattern.applyTo(turretLedBuffer);
          TurretLED.setData(turretLedBuffer);
          break;
        case Hub:
          hubPattern.applyTo(turretLedBuffer);
          TurretLED.setData(turretLedBuffer);
          break;
        case Point:
          pointPattern.applyTo(turretLedBuffer);
          TurretLED.setData(turretLedBuffer);
          break;
        case Red:
          redPattern.applyTo(turretLedBuffer);
          TurretLED.setData(turretLedBuffer);
          break;
        case Blue:
          bluePattern.applyTo(turretLedBuffer);
          TurretLED.setData(turretLedBuffer);
          break;
        case EndgameRed:
          redEndPattern.applyTo(endgameBuffer);
          TurretLED.setData(endgameBuffer);
          break;
        case EndgameBlue:
          blueEndPattern.applyTo(endgameBuffer);
          TurretLED.setData(endgameBuffer);
          break;
        case PrepareRed:
          redPreparePattern.applyTo(turretLedBuffer);
          TurretLED.setData(turretLedBuffer);
          break;
        case PrepareBlue:
          bluePreparePattern.applyTo(turretLedBuffer);
          TurretLED.setData(turretLedBuffer);
          break;
        case Off:
          offPattern.applyTo(turretLedBuffer);
          TurretLED.setData(turretLedBuffer);
        default:
          break;  
      }
    }
  
    public static void startEndgame() {
      endgameTimer.restart();
  }

}