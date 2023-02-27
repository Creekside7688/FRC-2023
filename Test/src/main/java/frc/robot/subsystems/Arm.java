// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.ARM_OFFSET;
import static frc.robot.Constants.ArmConstants.DEGREES_PER_PULSE;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final WPI_VictorSPX armMotorA;
    private final WPI_VictorSPX armMotorB;

    public double setPoint;

    private final MotorControllerGroup motors;

    private final Encoder encoder;

    public Arm() {
        armMotorA = new WPI_VictorSPX(9);
        armMotorB = new WPI_VictorSPX(10);
        armMotorA.setNeutralMode(NeutralMode.Brake);
        armMotorB.setNeutralMode(NeutralMode.Brake);

        motors = new MotorControllerGroup(armMotorA, armMotorB);
        motors.setInverted(false);

        encoder = new Encoder(5, 6, false);
        encoder.setDistancePerPulse(DEGREES_PER_PULSE);
    }

    public void resetEncoder() {
        encoder.reset();
    }

    public void stop() {
        motors.stopMotor();
    }

    public void turn(double speed) {
        motors.set(speed);
    }

    public double getEncoderAbsoluteDegrees() {
        return encoder.getDistance() + ARM_OFFSET;
    }

    @Override
    public void periodic() {
    }
}