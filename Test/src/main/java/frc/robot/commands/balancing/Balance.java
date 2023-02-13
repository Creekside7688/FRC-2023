// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.balancing;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Balance extends CommandBase {
  private final DriveTrain driveTrain;
  private PIDControllerDelegator pidController;

  private boolean isBalanced;

  public Balance(DriveTrain d) {
    driveTrain = d;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController = new PIDControllerDelegator(0, driveTrain);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double driveTrainPitch = driveTrain.getPitch();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isBalanced;
  }
}
