// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

public class Drive extends CommandBase {
    private final DriveTrain driveTrain;

    public Drive(DriveTrain d) {
        driveTrain = d;
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        driveTrain.arcadeDrive(RobotContainer.driverController.getRawAxis(XboxController.Axis.kLeftY.value), RobotContainer.driverController.getRawAxis(XboxController.Axis.kRightX.value)*-1);
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.resetEncoders();
        driveTrain.Stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}