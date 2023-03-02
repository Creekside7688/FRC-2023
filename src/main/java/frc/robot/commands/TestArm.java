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

public class TestArm extends CommandBase {
    double b = -60;
    private final Arm arm;
    private final Wrist myWrist;
    private PIDController pidController;
    private PIDController wristPidController;
    private Joystick joystick = new Joystick(0);
    private double pidOutput;
    private double minPower;
    private double speed;

    public TestArm(Arm arm, Wrist wrist) {
        this.arm = arm;
        myWrist = wrist;
        addRequirements(myWrist);
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        pidController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
        wristPidController = new PIDController(0.03, 0, 0);
        arm.resetEncoder();
        myWrist.resetEncoder();
        
        
        pidController.setTolerance(3, 0.1);
        wristPidController.setTolerance(3,0.1);
        b = -60;
    }

    @Override
    public void execute() {
        
        //System.out.println();
        pidController.setSetpoint(b);

        if(joystick.getRawButton(XboxController.Button.kRightBumper.value)){
            b += 0.4 ;
            if(b>0){
                b = 0;
            }
            pidController.setSetpoint(b);
        } else if(joystick.getRawAxis(XboxController.Axis.kRightTrigger.value) == 1){
            b -= 0.4;

            if(b < -90){
                b = -90;
            }
            pidController.setSetpoint(b);

        } else {
           //pidController.setSetpoint(b);
           
        }
               
        minPower = -Math.cos(Math.toRadians(arm.getEncoderAbsoluteDegrees()) + Math.PI / 6) * ArmConstants.KG;
        pidOutput = pidController.calculate(arm.getEncoderAbsoluteDegrees());
        //stalling speed
        speed = MathUtil.clamp(pidOutput + minPower, -0.3, 0.13)*-1;
        SmartDashboard.putNumber("B", b);
        //SmartDashboard.putNumber("minimum power", minPower);
        //System.out.println(arm.getEncoder());
        //SmartDashboard.putNumber("pid output", MathUtil.clamp(pidOutput + minPower, 0.0, 0.3) * -1);
        wristPidController.setSetpoint(arm.getEncoder());
        myWrist.turn(wristPidController.calculate(myWrist.getDegrees()));
        arm.turn(speed);
    }

    @Override
    public void end(boolean interrupted) {
       
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
