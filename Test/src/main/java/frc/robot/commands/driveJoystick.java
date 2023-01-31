// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveTrain;

public class driveJoystick extends CommandBase {
  private final DriveTrain m_DriveTrain;

  /** Creates a new driveJoystick. */
  public driveJoystick(DriveTrain d) {
    m_DriveTrain = d;
    addRequirements(m_DriveTrain);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_DriveTrain.arcadeDrive(RobotContainer.m_driverController.getRawAxis(OperatorConstants.LEFT_Y_AXIS), RobotContainer.m_driverController.getRawAxis(OperatorConstants.RIGHT_X_AXIS));
    /*
     * System.out.print("Left cm: "+m_DriveTrain.getLeft_enc_dis());
     * System.out.print("Right cm: " +m_DriveTrain.getRight_enc_dis());
     * System.out.print("Gyro yaw: "+m_DriveTrain.get_Yaw());
     * System.out.print("Gyro pitch: "+ m_DriveTrain.get_Pitch());
     * 
     * 
     */

    //  System.out.println("Gyro: (" + m_DriveTrain.getRoll() + ", " + m_DriveTrain.getPitch() + ", " + m_DriveTrain.getYaw() + ")");

     double[] encoderDistance = m_DriveTrain.getEncoders();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.Stop();
    m_DriveTrain.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}