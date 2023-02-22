// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

public class TestArm extends CommandBase {
  /** Creates a new TestArm. */
  private final Arm myArm;
  private PIDController pidController;
  private double encoderData;
  private double pidOutput;

  public TestArm(Arm a) {
    myArm = a;
    addRequirements(myArm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myArm.resetEncoder();
    pidController = new PIDController(ArmConstants.KP, ArmConstants.KI, ArmConstants.KD);
    pidController.setSetpoint(ArmConstants.SETPOINT);
    pidController.setTolerance(10,0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Encoder data", myArm.getArmEncoder());
    SmartDashboard.putNumber("position error", pidController.getPositionError());
    SmartDashboard.putNumber("raw output", pidOutput);

    encoderData = myArm.getArmEncoder();
    pidOutput = pidController.calculate(encoderData);
    myArm.run(MathUtil.clamp(pidOutput, -0.3, 0.3));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myArm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }
}
