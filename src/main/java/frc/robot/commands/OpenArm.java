// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class OpenArm extends CommandBase {
    private final Arm arm;
    private final PIDController pidController;
    private double direction = -1;
    private Wrist wrist;

    public OpenArm(Arm arm, Wrist wrist) {
        this.arm = arm;
        this.wrist = wrist;

        pidController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
        pidController.setSetpoint(140);
        pidController.setTolerance(2);

        addRequirements(arm);
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        arm.resetEncoder();
        wrist.resetEncoder();
        wrist.turn(0.07);
    }

    @Override
    public void execute() {
        double output = pidController.calculate(arm.getEncoderAbsoluteDegrees());
        arm.turn(MathUtil.clamp(output, 0, 0.35) * direction);
        SmartDashboard.putNumber("degrees", arm.getEncoderAbsoluteDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        arm.turn(.01);
        arm.resetEncoder();
        SmartDashboard.putString("is finished?", "yes");
    }

    @Override
    public boolean isFinished() {
        if(arm.getEncoderAbsoluteDegrees() > 135) {
            direction = 0;
        }

        return arm.getEncoderAbsoluteDegrees() > 250;
    }
}
