// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HandMotor;
import frc.robot.subsystems.Hand;

public class CloseH extends CommandBase {
  /** Creates a new CloseH. */
  private final Hand hk;
  public CloseH(Hand hand) {
    hk = hand;
    addRequirements(hk);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hk.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hk.runMotor(HandMotor.H_CLOSESPEED);
    hk.get_encoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hk.runMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
