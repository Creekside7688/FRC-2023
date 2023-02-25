// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.OldArm;

public class TestArm extends CommandBase {
    /** Creates a new TestArm. */
    private final OldArm myArm;
    private PIDController pidController;
    private double encoderData;
    private double pidOutput;

    public TestArm(OldArm a) {
        myArm = a;
        addRequirements(myArm);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        myArm.resetEncoder();
        pidController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
        pidController.setSetpoint(ArmConstants.SETPOINT);
        pidController.setTolerance(3, 0.1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //encoderData = myArm.getArmEncoder();
        //pidOutput = pidController.calculate(encoderData);
         //double g = Math.cos(encoderData*(2*Math.PI/2048));
        SmartDashboard.putNumber("Encoder value", encoderData);
        SmartDashboard.putNumber("pid output", MathUtil.clamp(pidOutput, 0, 0.25)*-1);
        //myArm.run(MathUtil.clamp(pidOutput, 0, 0.25)*-1);
        myArm.run(-0.25);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        myArm.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
        //return pidController.atSetpoint();
    }
}
