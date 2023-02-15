// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import static frc.robot.Constants.OperatorConstants.*;
import frc.robot.subsystems.DriveTrain;

public class DriveJoystick extends CommandBase {
  private final DriveTrain driveTrain;

  public DriveJoystick(DriveTrain d) {
    driveTrain = d;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.arcadeDrive(RobotContainer.driverController.getRawAxis(LEFT_Y_AXIS), RobotContainer.driverController.getRawAxis(RIGHT_X_AXIS));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.resetEncoders();
    driveTrain.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}