// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.OperatorConstants;

public class driveJoystic extends CommandBase {
  /** Creates a new driveJoystic. */
  private final DriveTrain m_DriveTrain;
  private final Joystick joystick = new Joystick(OperatorConstants.JOYSTICK_PORT);
  public driveJoystic(DriveTrain d) {
    m_DriveTrain = d;
    addRequirements(m_DriveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_DriveTrain.reset_encoders();
    m_DriveTrain.reset_Yaw();;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveTrain.arcadeDrive(joystick.getRawAxis(OperatorConstants.LEFT_Y_AXIS), joystick.getRawAxis(OperatorConstants.RIGHT_X_AXIS));
    System.out.print("Left cm: "+m_DriveTrain.getLeft_enc_dis());
    System.out.print("Right cm: " +m_DriveTrain.getRight_enc_dis());
    System.out.print("Gyro yaw: "+m_DriveTrain.get_Yaw());
    System.out.print("Gyro pitch: "+ m_DriveTrain.get_Pitch());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.Stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
