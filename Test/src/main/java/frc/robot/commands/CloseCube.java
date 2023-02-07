// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//*** WANRING THIS FUNCTION SHOULD ONLY BE CALLED AFTER CALLING THE OPEN COMMAND ***/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;
import frc.robot.Constants.HandMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CloseCube extends CommandBase {
  /** Creates a new CloseCube. */
  private final Hand hand;
  private final double distanceToClose;
  private final double pulsesToTravelDistance;

  public CloseCube(Hand h) {
    hand = h;
    distanceToClose = HandMotor.DISTANCE_TO_CLOSE_CUBE_CM;
    pulsesToTravelDistance = distanceToClose/HandMotor.CM_PER_PULSE;
    addRequirements(hand);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    while(hand.get_encoder() < pulsesToTravelDistance){
      SmartDashboard.putNumber("Hand encoder pos: ", hand.get_encoder());
      hand.runMotor(HandMotor.H_CLOSESPEED);
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}