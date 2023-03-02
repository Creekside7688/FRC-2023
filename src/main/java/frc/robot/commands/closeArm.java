// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class closeArm extends CommandBase {
    private final Arm myArm;
    private double speedMulti = 1;
    private final Wrist wrist;

    public closeArm(Arm arm, Wrist wrist) {
        myArm = arm;
        this.wrist = wrist;        
        addRequirements(myArm);
        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        myArm.resetEncoder();
        wrist.resetEncoder();
        wrist.turn(0.05);
    }

    @Override
    public void execute() {
        myArm.turn(0.3 * speedMulti);
    }

    @Override
    public void end(boolean interrupted) {
        myArm.turn(0);
    }

    @Override
    public boolean isFinished() {
        if(myArm.getEncoderAbsoluteDegrees() < -140) {
            speedMulti = 0;
            return true;
        }
        return false;
    }
}
