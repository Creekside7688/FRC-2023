// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HandMotorConstants;
import frc.robot.subsystems.Hand;

public class OpenH extends CommandBase {
    /** Creates a new OpenH. */
    private final Hand hd;

    public OpenH(Hand h) {
        hd = h;
        addRequirements(hd);

        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hd.runClaw(HandMotorConstants.H_OPENSPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hd.runClaw(0);
        hd.resetEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return false;
        return !hd.getLimitSwitch();
    }
}
