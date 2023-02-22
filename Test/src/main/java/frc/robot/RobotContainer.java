// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.HandMotor;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CloseCone;
import frc.robot.commands.CloseCube;
import frc.robot.commands.CloseH;
import frc.robot.commands.DriveDistance;
import frc.robot.commands.Limelight;
import frc.robot.commands.driveJoystick;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.OpenH;
import frc.robot.commands.TestArm;
import frc.robot.subsystems.Hand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class RobotContainer {
  public static Joystick m_driverController = new Joystick(OperatorConstants.JOYSTICK_PORT);

  private final Trigger rb_Button = new JoystickButton(m_driverController, OperatorConstants.RB_BUTTON);
  private final Trigger x_Button = new JoystickButton(m_driverController, OperatorConstants.X_BUTTON);
  private final Trigger y_Button = new JoystickButton(m_driverController, OperatorConstants.Y_BUTTON);
  private final Trigger a_Button = new JoystickButton(m_driverController, OperatorConstants.A_BUTTON);
  private final Trigger b_button = new JoystickButton(m_driverController, OperatorConstants.B_BUTTON);

  private final DriveTrain dt = new DriveTrain();
  private final Hand hd = new Hand();
  private final Arm am = new Arm();
  private final OpenH open = new OpenH(hd);
  private final CloseCube closeCone = new CloseCube(hd, HandMotor.H_CLOSE_CONE_SPEED, HandMotor.H_CLOSE_CONE_ENDSPEED, HandMotor.HOLD_TIME_CONE);
  private final CloseCube closeCube = new CloseCube(hd, HandMotor.H_CLOSE_CUBE_SPEED, HandMotor.H_CLOSE_CUBE_ENDSPEED, HandMotor.HOLD_TIME_CUBE);
  private final TestArm testMyArm = new TestArm(am);
  private final driveJoystick drive = new driveJoystick(dt);
  private final Limelight camera = new Limelight(dt);

  private final DriveDistance driveDistance = new DriveDistance(100, dt);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    rb_Button.whileTrue(drive);
    x_Button.whileTrue(camera);
    y_Button.onTrue(closeCone);
    a_Button.onTrue(testMyArm);
    b_button.onTrue(closeCube);
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