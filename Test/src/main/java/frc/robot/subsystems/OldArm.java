// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class OldArm extends SubsystemBase {
    /** Creates a new Arm. */
    private final WPI_VictorSPX armMotorA;
    private final WPI_VictorSPX armMotorB;
    private final Encoder armEncoder;

    public OldArm() {
        // change device number later
        armMotorA = new WPI_VictorSPX(9);
        armMotorB = new WPI_VictorSPX(10);

        armMotorA.setNeutralMode(NeutralMode.Brake);
        armMotorB.setNeutralMode(NeutralMode.Brake);

        armEncoder = new Encoder(5, 6, false, EncodingType.k4X);
        armEncoder.setDistancePerPulse(Constants.ArmConstants.DEGREE_PER_PULS);
    }

    public void resetEncoder() {
        armEncoder.reset();
    }

    public void stop() {
        armMotorA.set(0);
        armMotorB.set(0);
        armMotorA.setVoltage(0);
        armMotorB.setVoltage(0);
    }

    public void run(double speed) {
        armMotorA.set(speed);
        armMotorB.set(speed);
    }

    public double getArmEncoder() {
        return armEncoder.get();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}