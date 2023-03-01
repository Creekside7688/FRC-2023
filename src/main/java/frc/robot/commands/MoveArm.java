// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class MoveArm extends CommandBase {
    private final Arm arm;
    private final PIDController pidController;
    private double targetAngle;

    public MoveArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);

        pidController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
        pidController.setSetpoint(0);
        pidController.setTolerance(3, 0.1);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        pidController.setSetpoint(targetAngle);
        double minPower = -Math.cos(Math.toRadians(arm.getEncoderAbsoluteDegrees()) + Math.PI / 6) * ArmConstants.KG;
        double output = pidController.calculate(arm.getEncoderAbsoluteDegrees());

        arm.turn(MathUtil.clamp(output + minPower, -0.3, 0.15) * -1);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }
}
