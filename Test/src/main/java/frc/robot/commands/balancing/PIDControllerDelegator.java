// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * PID controller class to make the code look cleaner and also to make us look more professional.
 */

package frc.robot.commands.balancing;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveTrain;

public class PIDControllerDelegator {
  private PIDController pidController;
  private final DriveTrain driveTrain;

  private double kP = PIDConstants.kP;
  private double kI = PIDConstants.kP;
  private double kD = PIDConstants.kP;

  private double distance = 0;

  public PIDControllerDelegator(double distance, DriveTrain d) {
    this.distance = distance;
    this.driveTrain = d;

    pidController = new PIDController(kP, kI, kD);

    driveTrain.resetEncoders();

    pidController.setSetpoint(this.distance);
    pidController.setTolerance(2);
  }

  public double calculate() {
    double encoderDistance = driveTrain.getEncoderAverage();
    double output = pidController.calculate(encoderDistance);
    return MathUtil.clamp(-output, -0.4, 0.4);
  }

  public void setSetpoint(double setpoint) {
    pidController.setSetpoint(setpoint);
  }
}
