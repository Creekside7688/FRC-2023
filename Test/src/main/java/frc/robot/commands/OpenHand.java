// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.HandMotor;
import frc.robot.subsystems.Hand;

public class OpenHand extends CommandBase {
  /** Creates a new OpenH. */
  private final Hand hand;
  public OpenHand(Hand h) {
    hand = h;
    addRequirements(hand);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hand.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hand.runMotor(HandMotor.H_OPENSPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hand.runMotor(0);
    hand.resetEncoder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !hand.getLimitSwitch();
  }
}