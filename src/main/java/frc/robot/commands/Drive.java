// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RGB;

public class Drive extends CommandBase {
    private final DriveTrain driveTrain;
    private final RGB leds;
    public Drive(DriveTrain d, RGB l) {
        driveTrain = d;
        leds = l;
        addRequirements(leds);
        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        double speed = RobotContainer.driverController.getRawAxis(XboxController.Axis.kLeftY.value);
        double rotation = RobotContainer.driverController.getRawAxis(XboxController.Axis.kRightX.value);
        if(Math.abs(speed) < 0.15) {
            speed = 0;
        }
        if(Math.abs(rotation) < 0.15) {
            rotation = 0;
        }

        // speed = Math.pow(speed, 2) * Math.signum(spe // Squareaq the input for greater control at lower speeds.

        driveTrain.arcadeDrive(speed, rotation);
        leds.speedColor(speed, rotation);
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