// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.LimeLightConstants;
import frc.robot.subsystems.DriveTrain;

public class AprilTagAlign extends CommandBase {
    private final DriveTrain driveTrain;
    private double rotateOutput;
    private double speedOutput;
    private final double rotateSpeed = LimeLightConstants.ROTATE_SPEED;
    private final double driveSpeed = LimeLightConstants.DRIVE_SPEED;
    private final double rotateMultiplier = LimeLightConstants.ROTATE_MULTIPLIER;

    public AprilTagAlign(DriveTrain dt) {
        driveTrain = dt;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        rotateOutput = driveTrain.getXLimelight() * rotateMultiplier;
        speedOutput = 11 - driveTrain.getTargetArea();
        SmartDashboard.putNumber("rotateOutput", rotateOutput);
        driveTrain.arcadeDrive(MathUtil.clamp(speedOutput, 0, driveSpeed) * -1, MathUtil.clamp(rotateOutput, rotateSpeed * -1, rotateSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        // return driveTrain.getTargetArea() > 61 && driveTrain.getTargetArea() < 59;
        return driveTrain.getTargetArea() > 10.5;
    }
}
