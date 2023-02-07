// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CloseH;
import frc.robot.commands.Balance;
import frc.robot.commands.driveJoystick;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.OpenH;
import frc.robot.subsystems.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  public static Joystick m_driverController = new Joystick(OperatorConstants.JOYSTICK_PORT);

  private final Trigger rb_Button = new JoystickButton(m_driverController, OperatorConstants.RB_BUTTON);
  private final Trigger x_Button = new JoystickButton(m_driverController, OperatorConstants.X_BUTTON);
  private final Trigger y_Button = new JoystickButton(m_driverController, OperatorConstants.Y_BUTTON);
  private final Trigger a_Button = new JoystickButton(m_driverController, OperatorConstants.A_BUTTON);

  private final DriveTrain dt = new DriveTrain();
  private final Hand hd = new Hand();
  private final OpenH open = new OpenH(hd);
  private final CloseH close = new CloseH(hd);
  private final driveJoystick drive = new driveJoystick(dt);

  // private final DriveDistance driveDistance = new DriveDistance(50, dt);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    rb_Button.whileTrue(drive);
    x_Button.whileTrue(open);
    y_Button.whileTrue(close);
    a_Button.whileTrue(new Balance(100, dt));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}