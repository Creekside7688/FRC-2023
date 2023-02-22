// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final WPI_VictorSPX armMotor;
  private final Encoder armEncoder; 
  public Arm() {
    armMotor = new WPI_VictorSPX(7);
    armEncoder = new Encoder(5, 6,true,EncodingType.k4X);
  }

  public void resetEncoder(){
    armEncoder.reset();
  }

  public void stop(){
    armMotor.set(0);
  }

  public void run(double speed){
    armMotor.set(speed);
  }

  public double getArmEncoder(){
    return armEncoder.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}