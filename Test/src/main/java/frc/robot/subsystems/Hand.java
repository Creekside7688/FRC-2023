// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HandMotor;

public class Hand extends SubsystemBase {
  /** Creates a new Hand. */
  private final CANSparkMax handMotor = new CANSparkMax(HandMotor.HAND_PORT, MotorType.kBrushless);
  private final RelativeEncoder encoder = handMotor.getEncoder();
  private final DigitalInput limitSwitch = new DigitalInput(4);

  public Hand() {
    handMotor.setIdleMode(IdleMode.kBrake);
  }

  public void runMotor(double speed){
    handMotor.set(speed);

  }
  public double getEncoder() {
    return encoder.getPosition();

  }
  
  public boolean getLimitSwitch() {
    return limitSwitch.get();

  }
  
  public void resetEncoder() {
    encoder.setPosition(0);

  }

  public void setCoast() {
    handMotor.setIdleMode(IdleMode.kCoast);
  }

  public void setBreak() {
    handMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
