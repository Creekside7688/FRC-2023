// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
    private final DriveTrain driveTrain;
    private final double distance;

    public DriveDistance(DriveTrain driveTrain, double distance) {
        this.driveTrain = driveTrain;
        this.distance = distance;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.resetEncoders();
        
        driveTrain.setBrake();
    }

    @Override
    public void execute() {
        driveTrain.arcadeDrive(-0.4, 0);
        SmartDashboard.putNumber("encoder drive distance", driveTrain.getEncoderAverage());
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.Stop();
    }

    @Override
    public boolean isFinished() {
        return driveTrain.getEncoderAverage() < -distance;
    }
}
