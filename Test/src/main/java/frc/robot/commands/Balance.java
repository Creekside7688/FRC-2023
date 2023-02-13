// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PIDConstants;
import frc.robot.subsystems.DriveTrain;

public class Balance extends CommandBase {
  private final DriveTrain driveTrain;
  private PIDController pidController;
  private final double distance;

  private final boolean finished = false;


  public Balance(double distance, DriveTrain d) {
    this.distance = distance;
    driveTrain = d;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);

    pidController.setSetpoint(distance);
    pidController.setTolerance(2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = 0;

    double absolutePitch = Math.abs(driveTrain.getPitch());
    double encoderDistance = driveTrain.getEncoderAverage();

    if(absolutePitch > 4) {
      // double sign = Math.signum(driveTrain.getPitch());


      output = pidController.calculate(encoderDistance);
      driveTrain.arcadeDrive(MathUtil.clamp(-output, -0.4, 0.4), 0);
    }

    SmartDashboard.putNumber("Distance :", encoderDistance);
    SmartDashboard.putNumber("Motor Output :", MathUtil.clamp(-output, -0.4, 0.4));
    SmartDashboard.putNumber("Position Error", pidController.getPositionError());
    SmartDashboard.putBoolean("Finished", pidController.atSetpoint());
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
