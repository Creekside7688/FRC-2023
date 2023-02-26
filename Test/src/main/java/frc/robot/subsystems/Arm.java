// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Arm extends SubsystemBase {
    private final WPI_VictorSPX armMotorA;
    private final WPI_VictorSPX armMotorB;

    private final MotorControllerGroup motors;

    private final Encoder armEncoder;

    public Arm() {
        armMotorA = new WPI_VictorSPX(9);
        armMotorB = new WPI_VictorSPX(10);

        armMotorA.setNeutralMode(NeutralMode.Brake);
        armMotorB.setNeutralMode(NeutralMode.Brake);

        motors = new MotorControllerGroup(armMotorA, armMotorB);

        armEncoder = new Encoder(5, 6, false);
        armEncoder.setDistancePerPulse(Constants.ArmConstants.DEGREE_PER_PULSE);
    }

    public void resetEncoder() {
        armEncoder.reset();
    }

    public void stop() {
        motors.stopMotor();
    }

    public void turn(double speed) {
        motors.set(speed);
    }

    public double getEncoderPosition() {
        return armEncoder.get();
    }

    public double getEncoderAsDegrees() {
        throw new UnsupportedOperationException("Brennan pls calculate this pls pls pls");
    }

    @Override
    public void periodic() {
    }
}