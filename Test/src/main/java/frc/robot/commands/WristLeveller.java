// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.WristConstants.DEGREES_PER_ROTATION;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class WristLeveller extends CommandBase {
    private final Wrist wrist;
    private final Arm arm;

    private final RelativeEncoder wristEncoder;

    public WristLeveller(Wrist wrist, Arm arm) {
        this.wrist = wrist;
        this.arm = arm;

        wristEncoder = wrist.getEncoder();
        wristEncoder.setPositionConversionFactor(DEGREES_PER_ROTATION);

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double armAngle = 360 - (arm.getEncoderAbsoluteDegrees());
        double targetAngle = 180 - (90 - armAngle);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
