// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.random.RandomGenerator.ArbitrarilyJumpableGenerator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class OpenArm extends CommandBase {
    private final Arm myArm;
    private final PIDController pidController;
    private double direction = -1;
    private double currentPos = 0;
    private Wrist myWrist;
    

    public OpenArm(Arm arm, Wrist wrist) {
        myArm = arm;
        myWrist = wrist;

        pidController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
        pidController.setSetpoint(140);
        pidController.setTolerance(2);

        addRequirements(arm);
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        myArm.resetEncoder();
        //currentPos = myArm.getEncoderAbsoluteDegrees();
        //wrist.resetEncoder();
        //wrist.turn(0.07);
        pidController.reset();
        
        SmartDashboard.putBoolean("state initialize", true);
    }

    @Override
    public void execute() {
        currentPos = myArm.getEncoderAbsoluteDegrees();
        double output = pidController.calculate(currentPos);
        myArm.turn(MathUtil.clamp(output, 0, 0.35) * direction);
        SmartDashboard.putNumber("arm speed", output);
        SmartDashboard.putNumber(" arm degrees", currentPos);
    }

    @Override
    public void end(boolean interrupted) {
        myArm.stop();
        SmartDashboard.putBoolean("state initialize", false);
        myArm.resetEncoder();
        
    }

    @Override
    public boolean isFinished() {
        //if(currentPos > 135) {
            
        //}

        SmartDashboard.putNumber("end condition", currentPos);
        return currentPos > 135;
    }
}
