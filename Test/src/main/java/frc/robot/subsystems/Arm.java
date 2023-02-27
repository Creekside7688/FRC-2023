// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class Arm extends SubsystemBase {
    private final WPI_VictorSPX armMotorA;
    private final WPI_VictorSPX armMotorB;

    private final MotorControllerGroup motors;

    private final DutyCycleEncoder encoder;

    public Arm() {
        armMotorA = new WPI_VictorSPX(9);
        armMotorB = new WPI_VictorSPX(10);

        armMotorA.setNeutralMode(NeutralMode.Brake);
        armMotorB.setNeutralMode(NeutralMode.Brake);

        motors = new MotorControllerGroup(armMotorA, armMotorB);

        encoder = new DutyCycleEncoder(5);
        encoder.reset();
        encoder.setDutyCycleRange(1.0 / 1024.0, 1023.0 / 1024.0);
        encoder.setDistancePerRotation(360);
    }

    public void stop() {
        motors.stopMotor();
    }

    public void turn(double speed) {
        motors.set(speed);
    }

    /**
     * Gets the rotation of the arm in degrees since the last reset. Use this to get the relative rotation.
     * 
     * @return The relative rotation of the arm in degrees since the last reset.
     */
    public double getEncoderRelativeDegrees() {
        return encoder.getDistance();
    }

    /**
     * Gets the rotation of the arm in degrees.
     * 
     * @return The rotation of the arm in degrees.
     */
    public double getEncoderDegrees() {
        return encoder.getAbsolutePosition() * encoder.getDistancePerRotation();
    }

    public double getEncoderOffset() {
        return encoder.getPositionOffset() * encoder.getDistancePerRotation();
    }

    public void resetEncoder() {
        encoder.reset();
    }

    @Override
    public void periodic() {
    }
}