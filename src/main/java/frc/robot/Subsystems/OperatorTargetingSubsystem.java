// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OperatorTargetingSubsystem extends SubsystemBase {

    private final DoubleSubscriber xSubscriber;
    private final DoubleSubscriber ySubscriber;

    public Translation2d operatorTargetPos = new Translation2d();

    public OperatorTargetingSubsystem() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Operator Target");
        xSubscriber = table.getDoubleTopic("targetX").subscribe(-1.0);
        ySubscriber = table.getDoubleTopic("targetY").subscribe(-1.0);
    }

    @Override
    public void periodic() {
        double newX = xSubscriber.get();
        double newY = ySubscriber.get();

        if (newX >= 0 && newY >= 0) {
            operatorTargetPos = new Translation2d(newX, newY);
        }
    }

    public void clearTarget() {
        operatorTargetPos = new Translation2d();
    }
}