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
    /** Creates a new TestArm. */
    private final Arm arm;
    private PIDController pidController;
    private double encoderData;
    private double pidOutput;
    private double minPower;

    public TestArm(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.resetEncoder();
        pidController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
        pidController.setSetpoint(ArmConstants.SETPOINT);
        pidController.setTolerance(3, 0.1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        minPower = Math.cos(Math.toRadians(arm.getDegree())+Math.PI/6)*ArmConstants.KG;
        pidOutput = pidController.calculate(arm.getDegree());
        SmartDashboard.putNumber("Encoder value", encoderData);
        SmartDashboard.putNumber("minimum power", minPower);
        System.out.println(arm.getDegree());
        SmartDashboard.putNumber("pid output", MathUtil.clamp(pidOutput+minPower, 0.0, 0.3) * -1);
        arm.turn(MathUtil.clamp(pidOutput+minPower, 0.0, 0.3) * -1);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
