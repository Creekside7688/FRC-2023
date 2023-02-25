// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControlConstants;
import frc.robot.commands.Close;
import frc.robot.commands.DriveJoystick;
import frc.robot.commands.LimeLight;
import frc.robot.commands.OpenH;
import frc.robot.commands.TestArm;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.balancing.Balancer;
import frc.robot.commands.balancing.Search;
import frc.robot.subsystems.Hand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.HandMotorConstants.*;

public class RobotContainer {
    public static Joystick driverController = new Joystick(ControlConstants.JOYSTICK_PORT);

    private final Trigger lb_Button = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    private final Trigger lt_Trigger = new Trigger(RobotContainer::ltAsButton);

    private final Trigger rb_Button = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    private final Trigger rt_Trigger = new Trigger(RobotContainer::rtAsButton);

    private final Trigger a_Button = new JoystickButton(driverController, XboxController.Button.kA.value);
    private final Trigger b_Button = new JoystickButton(driverController, XboxController.Button.kB.value);
    private final Trigger x_Button = new JoystickButton(driverController, XboxController.Button.kX.value);
    private final Trigger y_Button = new JoystickButton(driverController, XboxController.Button.kY.value);

    private final Trigger ls_Button = new JoystickButton(driverController, XboxController.Button.kLeftStick.value);
    private final Trigger rs_Button = new JoystickButton(driverController, XboxController.Button.kRightStick.value);

    private final Arm myArm = new Arm();
    private final DriveTrain driveTrain = new DriveTrain();
    private final Hand hand = new Hand();
    // private final Close CloseCube = new Close(hand, H_CLOSE_CUBE_SPEED, H_CLOSE_CUBE_ENDSPEED, HOLD_TIME_CUBE);
    private final Close CloseCone = new Close(hand, H_CLOSE_CONE_SPEED, H_CLOSE_CONE_ENDSPEED, HOLD_TIME_CONE);
    private final LimeLight camera = new LimeLight(driveTrain);
    private final OpenH open = new OpenH(hand);
    private final TestArm testArm = new TestArm(myArm);

    private final DriveJoystick drive = new DriveJoystick(driveTrain);

    private final Search balancingSearcher = new Search(200, driveTrain);
    private final Balancer balancer = new Balancer(driveTrain);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();

        driveTrain.setDefaultCommand(drive);
    }

    private void configureBindings() {
        x_Button.onTrue(balancingSearcher.andThen(balancer.unless(() -> !balancingSearcher.runBalance)));
        y_Button.whileTrue(testArm);

        b_Button.onTrue(CloseCone);
        a_Button.onTrue(open);

        ls_Button.whileTrue(camera);
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

    private static final boolean ltAsButton() {
        return driverController.getRawAxis(XboxController.Axis.kLeftTrigger.value) > 0.5;
    }

    private static final boolean rtAsButton() {
        return driverController.getRawAxis(XboxController.Axis.kRightTrigger.value) > 0.5;
    }
}