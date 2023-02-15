// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//*** WARNING THIS FUNCTION SHOULD ONLY BE CALLED AFTER THE OPEN COMMAND ***/
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;
import static frc.robot.Constants.HandMotorConstants.*;

public class CloseCone extends CommandBase {
  private final Hand hand;
  private final double distanceToClose;
  private final double pulsesToTravelDistance;

  public CloseCone(Hand h) {
    hand = h;
    distanceToClose = DISTANCE_TO_CLOSE_CONE_CM;
    pulsesToTravelDistance = distanceToClose / CM_PER_PULSE;
    addRequirements(hand);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hand.resetEncoder();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hand.runMotor(H_CLOSESPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hand.runMotor(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Check if the motor has rotated farther than the target distance.
    return hand.getEncoder() > pulsesToTravelDistance;
  }
}
