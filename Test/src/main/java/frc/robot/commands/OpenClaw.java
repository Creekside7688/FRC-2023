// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HandConstants;
import frc.robot.subsystems.Claw;

public class OpenClaw extends CommandBase {
    private final Claw claw;

    public OpenClaw(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.runClaw(HandConstants.OPEN_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        claw.runClaw(0);
        claw.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return !claw.getLimitSwitch();
    }
}
