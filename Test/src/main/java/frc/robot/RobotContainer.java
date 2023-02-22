// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Close;
import frc.robot.commands.DriveJoystick;
import frc.robot.commands.LimeLight;
import frc.robot.commands.OpenH;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.balancing.Balancer;
import frc.robot.commands.balancing.Search;
import frc.robot.subsystems.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.HandMotorConstants.*;

public class RobotContainer {
    public static Joystick driverController = new Joystick(OperatorConstants.JOYSTICK_PORT);

    private final Trigger rb_Button = new JoystickButton(driverController, OperatorConstants.RB_BUTTON);
    private final Trigger x_Button = new JoystickButton(driverController, OperatorConstants.X_BUTTON);
    private final Trigger y_Button = new JoystickButton(driverController, OperatorConstants.Y_BUTTON);
    private final Trigger a_Button = new JoystickButton(driverController, OperatorConstants.A_BUTTON);
    private final Trigger b_Button = new JoystickButton(driverController, OperatorConstants.B_BUTTON);
    private final Trigger lb_Button = new JoystickButton(driverController, OperatorConstants.LB_BUTTON);

    private final DriveTrain driveTrain = new DriveTrain();
    private final Hand hand = new Hand();
    private final Close CloseCube = new Close(hand, H_CLOSE_CUBE_SPEED, H_CLOSE_CUBE_ENDSPEED, HOLD_TIME_CUBE);
    private final Close CloseCone = new Close(hand, H_CLOSE_CONE_SPEED, H_CLOSE_CONE_ENDSPEED, HOLD_TIME_CONE);
    private final LimeLight camera = new LimeLight(driveTrain);
    private final OpenH open = new OpenH(hand);

    private final DriveJoystick drive = new DriveJoystick(driveTrain);

    private final Search balancingSearcher = new Search(200, driveTrain);
    private final Balancer balancer = new Balancer(driveTrain);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

    private void configureBindings() {
        rb_Button.whileTrue(drive);
        x_Button.onTrue(CloseCone);
        y_Button.onTrue(CloseCube);
        lb_Button.onTrue(open);
        b_Button.whileTrue(camera);
        a_Button.onTrue(balancingSearcher.andThen(balancer.unless(() -> !balancingSearcher.runBalance)));
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