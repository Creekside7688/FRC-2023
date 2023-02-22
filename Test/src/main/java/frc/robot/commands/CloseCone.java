// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//*** WARNING THIS FUNCTION SHOULD ONLY BE CALLED AFTER THE OPEN COMMAND ***/
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;
import frc.robot.Constants.HandMotor;

public class CloseCone extends CommandBase {
  /** Creates a new CloseCube. */
  private final Hand hand;
  private final Timer time = new Timer();
  private double previousPos = 0;

  public CloseCone(Hand h) {
    hand = h;
    addRequirements(hand);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hand.runMotor(HandMotor.H_CLOSE_CONE_SPEED);
    previousPos = hand.get_encoder();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //let the motor have little power so it will keep holding even when command ends
    hand.runMotor(HandMotor.H_CLOSE_CONE_ENDSPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //delay so that it can detect a difference between current encoder value and previousPos encoder value
    Timer.delay(HandMotor.H_DELAY_CHECK);
    
    //if the position of the motor hasnt changed, and 3 seconds have passed, end command
    if(hand.get_encoder() > previousPos - HandMotor.DEADZONE_OFFSET && hand.get_encoder() < previousPos + HandMotor.DEADZONE_OFFSET){
      //if 3 seconds have passed the method will return true ending the command
      return time.hasElapsed(HandMotor.HOLD_TIME_CONE);
    }else{
      //reset time so command ends when 3 seconds have passed since the motor position hasnt changed
      time.reset();
    }

    return false;
  }
}
