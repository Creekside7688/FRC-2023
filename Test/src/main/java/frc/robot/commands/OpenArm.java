// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ArmConstants;

public class OpenArm extends CommandBase {
    private final Arm arm;
    private final PIDController pidController;
    private double direction = -1;

    public OpenArm(Arm arm) {
        this.arm = arm;

        pidController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
        pidController.setSetpoint(225);
        pidController.setTolerance(2);

        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.resetEncoder();
    }

    @Override
    public void execute() {
        double output = pidController.calculate(arm.getEncoderAbsoluteDegrees());
        // arm.turn(0.35);
        arm.turn(MathUtil.clamp(output, 0, 0.28) * direction);
        SmartDashboard.putNumber("degrees", arm.getEncoderAbsoluteDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        arm.turn(.01);
        SmartDashboard.putString("is finished?", "yes");
    }

    @Override
    public boolean isFinished() {
        if(arm.getEncoderAbsoluteDegrees() > 130) {
            direction = 0.05;
        }

        return pidController.atSetpoint();
    }
}
