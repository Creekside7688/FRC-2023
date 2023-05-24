// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ControlConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.RGB;

public class Drive extends CommandBase {
    private final DriveTrain driveTrain;
    private final SlewRateLimiter limiter = new SlewRateLimiter(3);

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

        speed = MathUtil.applyDeadband(speed, ControlConstants.DEAD_BAND);
        rotation = MathUtil.applyDeadband(rotation, ControlConstants.DEAD_BAND);

        speed = Math.pow(speed, 3);
        rotation = Math.pow(rotation, 3);

        speed += Math.abs(speed) != 0 ? Math.signum(speed) * ControlConstants.OFFSET : 0.0;
        rotation += Math.abs(rotation) != 0 ? Math.signum(rotation) * ControlConstants.OFFSET : 0.0;

        speed = limiter.calculate(speed);
        rotation = limiter.calculate(rotation);

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