// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandMotorConstants;

public class Hand extends SubsystemBase {
    private final CANSparkMax Hmotor = new CANSparkMax(HandMotorConstants.HAND_PORT, MotorType.kBrushless);
    private final RelativeEncoder encoder = Hmotor.getEncoder(Type.kHallSensor, 42);
    private final DigitalInput limitSwitch = new DigitalInput(4);

    public Hand() {
        Hmotor.setIdleMode(IdleMode.kBrake);
    }

    public void runMotor(double speed) {
        Hmotor.set(speed);

    }

    public double get_encoder() {
        return encoder.getPosition();

    }

    public boolean get_limit_switch() {
        return limitSwitch.get();

    }

    public void reset_encoder() {
        encoder.setPosition(0);

    }

    public void setCoast() {
        Hmotor.setIdleMode(IdleMode.kCoast);

    }

    public void setBreak() {
        Hmotor.setIdleMode(IdleMode.kBrake);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
