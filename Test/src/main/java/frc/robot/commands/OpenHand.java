// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HandConstants;
import frc.robot.subsystems.Claw;

public class OpenHand extends CommandBase {
    private final Claw claw;

    public OpenHand(Claw claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        claw.runClaw(HandConstants.H_OPENSPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        claw.runClaw(0);
        claw.resetEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return false;
        return !claw.getLimitSwitch();
    }
}
