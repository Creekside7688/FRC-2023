// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControlConstants;
import frc.robot.commands.CloseHand;
import frc.robot.commands.Drive;
import frc.robot.commands.AprilTagAlign;
import frc.robot.commands.OpenHand;
import frc.robot.commands.TestArm;
import frc.robot.commands.WristLeveller;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.balancing.Balancer;
import frc.robot.commands.balancing.Search;
import frc.robot.subsystems.Claw;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.HandConstants.*;

public class RobotContainer {
    public static Joystick driverController = new Joystick(ControlConstants.JOYSTICK_PORT);

    private final Trigger lb_Button = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
    private final Trigger lt_Trigger = new Trigger(RobotContainer::ltAsButton);

    private final Trigger rb_Button = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
    private final Trigger rt_Trigger = new Trigger(RobotContainer::rtAsButton);

    private final Trigger ls_Button = new JoystickButton(driverController, XboxController.Button.kLeftStick.value);
    private final Trigger rs_Button = new JoystickButton(driverController, XboxController.Button.kRightStick.value);

    private final Trigger a_Button = new JoystickButton(driverController, XboxController.Button.kA.value);
    private final Trigger b_Button = new JoystickButton(driverController, XboxController.Button.kB.value);
    private final Trigger x_Button = new JoystickButton(driverController, XboxController.Button.kX.value);
    private final Trigger y_Button = new JoystickButton(driverController, XboxController.Button.kY.value);

    private final Arm arm = new Arm();
    private final DriveTrain driveTrain = new DriveTrain();
    private final Claw hand = new Claw();
    private final Wrist wrist = new Wrist();

    private final CloseHand closeHand = new CloseHand(hand, H_CLOSE_CONE_SPEED, H_CLOSE_CONE_ENDSPEED, HOLD_TIME_CONE);
    private final OpenHand openHand = new OpenHand(hand);

    private final WristLeveller levelWrist = new WristLeveller(wrist, arm);

    private final AprilTagAlign aprilTagAlign = new AprilTagAlign(driveTrain);
    private final TestArm testArm = new TestArm(arm);

    private final Drive drive = new Drive(driveTrain);

    private final Search balancingSearcher = new Search(200, driveTrain);
    private final Balancer balancer = new Balancer(driveTrain);

    public RobotContainer() {
        configureBindings();

        driveTrain.setDefaultCommand(drive);
        wrist.setDefaultCommand(levelWrist);
    }

    private void configureBindings() {
        x_Button.onTrue(balancingSearcher.andThen(balancer.unless(() -> !balancingSearcher.runBalance)));
        y_Button.onTrue(closeHand);
        b_Button.onTrue(openHand);
        a_Button.whileTrue(testArm);
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

    public static final boolean getDpadUp() {
        return driverController.getPOV() == 0;
    }

    public static final boolean getDpadDown() {
        return driverController.getPOV() == 180;
    }

    public static final boolean getDpadLeft() {
        return driverController.getPOV() == 270;
    }

    public static final boolean getDpadRight() {
        return driverController.getPOV() == 90;
    }
}