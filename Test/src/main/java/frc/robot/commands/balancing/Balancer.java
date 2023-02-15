// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.balancing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Balancer extends CommandBase {
    private final DriveTrain driveTrain;

    private boolean isBalanced = false;

    public Balancer(DriveTrain d) {
        driveTrain = d;
        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isBalanced = false;
        driveTrain.setBrake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double angleError = 0 - driveTrain.getPitch();

        double output = Math.min(angleError * 0.0075, 1);

        output = MathUtil.clamp(-output, -0.2, 0.2);

        driveTrain.arcadeDrive(output, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveTrain.setCoast();
        driveTrain.resetEncoders();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isBalanced;
    }
}
