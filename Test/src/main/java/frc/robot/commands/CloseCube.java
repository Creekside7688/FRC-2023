// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//*** WANRING THIS FUNCTION SHOULD ONLY BE CALLED AFTER CALLING THE OPEN COMMAND ***/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;
import static frc.robot.Constants.HandMotorConstants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CloseCube extends CommandBase {
  private final Hand hand;
  private final double distanceToClose;
  private final double pulsesToTravelDistance;

  public CloseCube(Hand h) {
    hand = h;
    distanceToClose = DISTANCE_TO_CLOSE_CUBE_CM;
    pulsesToTravelDistance = distanceToClose / CM_PER_PULSE;
    addRequirements(hand);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    while(hand.getEncoder() < pulsesToTravelDistance){
      SmartDashboard.putNumber("Hand encoder pos: ", hand.getEncoder());
      hand.runMotor(H_CLOSESPEED);
    }
  
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
