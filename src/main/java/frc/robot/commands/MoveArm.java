// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class MoveArm extends CommandBase {
    private double armTarget = -60;
    private final Arm arm;
    private final Wrist wrist;
    private PIDController pidController;
    private PIDController wristPIDController;
    private Joystick joystick = new Joystick(0);

    public MoveArm(Arm arm, Wrist wrist) {
        this.arm = arm;
        this.wrist = wrist;
        addRequirements(wrist);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        pidController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
        wristPIDController = new PIDController(0.03, 0, 0);
        arm.resetEncoder();
        wrist.resetEncoder();

        pidController.setTolerance(3, 0.1);
        wristPIDController.setTolerance(3, 0.1);
        armTarget = -60;
    }

    @Override
    public void execute() {

        // System.out.println();
        pidController.setSetpoint(armTarget);

        if(joystick.getRawButton(XboxController.Button.kRightBumper.value)) {
            armTarget += 0.4;
            if(armTarget > 0) {
                armTarget = 0;
            }

            pidController.setSetpoint(armTarget);

        } else if(joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) == 1) {
            armTarget -= 0.4;

            if(armTarget < -90) {
                armTarget = -90;
            }

            pidController.setSetpoint(armTarget);

        }

        double minPower = -Math.cos(Math.toRadians(arm.getEncoderAbsoluteDegrees()) + Math.PI / 6) * ArmConstants.KG;
        double armOutput = pidController.calculate(arm.getEncoderAbsoluteDegrees());

        double armSpeed = -MathUtil.clamp(armOutput + minPower, -0.3, 0.13);

        SmartDashboard.putNumber("B", armTarget);

        wristPIDController.setSetpoint(arm.getEncoder());

        double wristOutput = wristPIDController.calculate(wrist.getDegrees());

        wrist.turn(wristOutput);
        arm.turn(armSpeed);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
