// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class WristLeveller extends CommandBase {
    private final Wrist wrist;
    private final PIDController pidController;

    public WristLeveller(Wrist wrist, Arm arm) {
        this.wrist = wrist;

        pidController = new PIDController(0.05, 0, 0);

        pidController.setTolerance(3, 0.1);
        pidController.setSetpoint(-63);
        addRequirements(wrist);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        wrist.resetEncoder();
        wrist.turn(0);
        pidController.reset();

    }

    @Override
    public void execute() {
        double output = pidController.calculate(wrist.getDegrees());
        SmartDashboard.putNumber("wrist wspeed", MathUtil.clamp(output, -0.4, 0.4));
        wrist.turn(MathUtil.clamp(output, -0.5, 0.5));

        SmartDashboard.putNumber("wrist encoder value: ", wrist.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stop();
        wrist.resetEncoder();
        SmartDashboard.putString("finished wrist level:", "yes");
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
