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

public class TestArm extends CommandBase {
    private final Arm arm;
    private PIDController pidController;
    private double encoderData;
    private double pidOutput;
    private double minPower;

    public TestArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pidController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
        pidController.setSetpoint(225);
        pidController.setTolerance(3, 0.1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        minPower = -Math.cos(Math.toRadians(arm.getEncoderAbsoluteDegrees())+Math.PI/12)*ArmConstants.KG;
        pidOutput = pidController.calculate(arm.getEncoderAbsoluteDegrees());

        SmartDashboard.putNumber("Encoder value", arm.getEncoderAbsoluteDegrees());
        SmartDashboard.putNumber("minimum power", minPower);
        System.out.println(arm.getEncoderAbsoluteDegrees());
        SmartDashboard.putNumber("pid output", MathUtil.clamp(pidOutput+minPower, -0.3, 0)*-1);
        arm.turn(MathUtil.clamp(pidOutput+minPower, -0.3, 0)*-1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
