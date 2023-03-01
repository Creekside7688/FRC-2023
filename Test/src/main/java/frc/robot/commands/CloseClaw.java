// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.HandConstants.CLOSE_SPEED;
import static frc.robot.Constants.HandConstants.DEADZONE_OFFSET;
import static frc.robot.Constants.HandConstants.DELAY_CHECK;
import static frc.robot.Constants.HandConstants.HOLD_SPEED;
import static frc.robot.Constants.HandConstants.HOLD_TIME;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class CloseClaw extends CommandBase {
    private final Claw claw;
    private double previousPos = 0;
    private final Timer time = new Timer();

    public CloseClaw(Claw claw) {
        this.claw = claw;

        addRequirements(claw);
    }

    @Override
    public void initialize() {
        time.reset();
        time.start();
    }

    @Override
    public void execute() {
        claw.runClaw(CLOSE_SPEED);
        previousPos = claw.getClawEncoder();
    }

    @Override
    public void end(boolean interrupted) {
        // let the motor have little power so it will keep holding even when command ends
        claw.runClaw(HOLD_SPEED);
    }

    @Override
    public boolean isFinished() {
        // delay so that it can detect a difference between current encoder value and previousPos encoder value
        Timer.delay(DELAY_CHECK);

        // if the position of the motor hasnt changed, and 3 seconds have passed, end command
        if(claw.getClawEncoder() > previousPos - DEADZONE_OFFSET && claw.getClawEncoder() < previousPos + DEADZONE_OFFSET) {
            // if 3 seconds haved passed the method will return true ending the command
            return time.hasElapsed(HOLD_TIME);
        } else {
            // reset time so command ends when 3 seconds have passed since the motor position hasnt changed
            time.reset();
        }

        return false;
    }
}
