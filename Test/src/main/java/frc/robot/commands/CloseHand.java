// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;
import frc.robot.Constants.HandConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CloseHand extends CommandBase {
    private final Claw hand;
    private double previousPos = 0;
    private final Timer time = new Timer();
    private final double closeSpeed;
    private final double endSpeed;
    private final double holdTime;

    public CloseHand(Claw h, double cspeed, double espeed, double htime) {
        hand = h;
        closeSpeed = cspeed;
        endSpeed = espeed;
        holdTime = htime;

        addRequirements(hand);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        time.reset();
        time.start();

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hand.runClaw(closeSpeed);
        previousPos = hand.getClawEncoder();

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // let the motor have little power so it will keep holding even when command ends
        SmartDashboard.putString("end?", "yes");
        hand.runClaw(endSpeed);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // delay so that it can detect a difference between current encoder value and previousPos encoder value
        Timer.delay(HandConstants.H_DELAY_CHECK);

        // if the position of the motor hasnt changed, and 3 seconds have passed, end command
        if(hand.getClawEncoder() > previousPos - HandConstants.DEADZONE_OFFSET && hand.getClawEncoder() < previousPos + HandConstants.DEADZONE_OFFSET) {
            // if 3 seconds haved passed the method will return true ending the command
            return time.hasElapsed(holdTime);
        } else {
            // reset time so command ends when 3 seconds have passed since the motor position hasnt changed
            time.reset();
        }

        return false;
    }
}
