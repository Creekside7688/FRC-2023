// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;
import frc.robot.Constants.HandMotorConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Close extends CommandBase {
    /** Creates a new CloseCube. */
    private final Hand hand;
    private double previousPos = 0;
    private final Timer time = new Timer();
    private final double closeSpeed;
    private final double endSpeed;
    private final double holdTime;

    public Close(Hand h, double cspeed, double espeed, double htime) {
        hand = h;
        closeSpeed = cspeed;
        endSpeed = espeed;
        holdTime = htime;

        addRequirements(hand);
        // Use addRequirements() here to declare subsystem dependencies.
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
        hand.runMotor(closeSpeed);
        previousPos = hand.get_encoder();
        SmartDashboard.putString("end?", "no");
        SmartDashboard.putNumber("encoder pos: ", hand.get_encoder());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // let the motor have little power so it will keep holding even when command ends
        SmartDashboard.putString("end?", "yes");
        hand.runMotor(endSpeed);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // delay so that it can detect a difference between current encoder value and previousPos encoder value
        Timer.delay(HandMotorConstants.H_DELAY_CHECK);

        // if the position of the motor hasnt changed, and 3 seconds have passed, end command
        if(hand.get_encoder() > previousPos - HandMotorConstants.DEADZONE_OFFSET && hand.get_encoder() < previousPos + HandMotorConstants.DEADZONE_OFFSET) {
            // if 3 seconds haved passed the method will return true ending the command
            return time.hasElapsed(holdTime);
        } else {
            // reset time so command ends when 3 seconds have passed since the motor position hasnt changed
            time.reset();
        }

        return false;
    }
}
