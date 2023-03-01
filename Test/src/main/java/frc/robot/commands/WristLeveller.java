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
    private final Arm arm;
    private final PIDController pidController;

    public WristLeveller(Wrist wrist, Arm arm) {
        this.wrist = wrist;
        this.arm = arm;

        pidController = new PIDController(0.03, 0, 0);
       
        pidController.setTolerance(3,0.1);
        pidController.setSetpoint(-40);
        addRequirements(wrist);
        addRequirements(arm);
    }

    @Override
    public void initialize() { 
        wrist.resetEncoder();
    }

    @Override
    public void execute() {
        // double armAngle = 360 - (arm.getEncoderAbsoluteDegrees());
        // double targetAngle = 180 - (90 - armAngle)
        System.out.print(arm.getEncoderAbsoluteDegrees());
        if(arm.getEncoderAbsoluteDegrees()>280){
             double output = pidController.calculate(wrist.getDegrees());
             wrist.turn(MathUtil.clamp(output, -0.3, 0.3));
             
        }
       SmartDashboard.putNumber("wrist encoder value: ", wrist.getDegrees());
    }

    @Override
    public void end(boolean interrupted) {
        wrist.stop();
        wrist.resetEncoder();
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }
}
