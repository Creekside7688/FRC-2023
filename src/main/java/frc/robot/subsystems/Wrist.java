// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.WristConstants.DEGREES_PER_ROTATION;
import static frc.robot.Constants.WristConstants.WRIST_PORT;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
    private final CANSparkMax wristMotor = new CANSparkMax(WRIST_PORT, MotorType.kBrushless);
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder(Type.kHallSensor, 42);

    public Wrist() {
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristEncoder.setPosition(0);
        wristEncoder.setPositionConversionFactor(DEGREES_PER_ROTATION);
    }

    @Override
    public void periodic() {
    }

    public void turn(double speed) {
        wristMotor.set(speed);
    }

    public void stop() {
        wristMotor.stopMotor();
    }

    public void resetEncoder() {
        wristEncoder.setPosition(0);
    }

    public double getDegrees() {
        return wristEncoder.getPosition();
    }
}
