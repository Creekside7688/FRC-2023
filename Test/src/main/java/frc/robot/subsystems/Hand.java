// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.HandMotorConstants.*;

public class Hand extends SubsystemBase {
    private final CANSparkMax clawMotor = new CANSparkMax(HAND_PORT, MotorType.kBrushless);
    private final CANSparkMax wristMotor = new CANSparkMax(WRIST_PORT, MotorType.kBrushless);

    private final RelativeEncoder clawEncoder = clawMotor.getEncoder(Type.kHallSensor, 42);
    private final RelativeEncoder wristEncoder = wristMotor.getEncoder(Type.kHallSensor, 42);

    private final DigitalInput limitSwitch = new DigitalInput(4);

    public Hand() {
        clawMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setIdleMode(IdleMode.kBrake);
    }

    public void runClaw(double speed) {
        clawMotor.set(speed);
    }

    public double getClawEncoder() {
        return clawEncoder.getPosition();

    }

    public boolean getLimitSwitch() {
        return limitSwitch.get();

    }

    public void resetEncoder() {
        clawEncoder.setPosition(0);

    }

    public void setCoast() {
        clawMotor.setIdleMode(IdleMode.kCoast);

    }

    public void setBrake() {
        clawMotor.setIdleMode(IdleMode.kBrake);

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
