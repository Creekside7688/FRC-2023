// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveTrain;

public class DriveDistance extends CommandBase {
  private final DriveTrain m_DriveTrain;
  private PIDController pidController;
  private final double Distance;

  public DriveDistance(double distance, DriveTrain d) {
    m_DriveTrain = d;
    Distance = distance;
    addRequirements(m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveTrain.reset();
    pidController = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);
    pidController.setSetpoint(50);
    pidController.setTolerance(1, 3);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double encoderDistance = m_DriveTrain.getEncoderAverage();
    double output = pidController.calculate(encoderDistance);
    m_DriveTrain.arcadeDrive(output, 0);
    SmartDashboard.putNumber("Distance :", encoderDistance);
    SmartDashboard.putNumber("Motor Output :", output);

    SmartDashboard.putNumber("Position Error :", pidController.getPositionError());
    // SmartDashboard.putNumber("kI", pidController.getI());
    // SmartDashboard.putNumber("kD", pidController.getD());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pidController.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
