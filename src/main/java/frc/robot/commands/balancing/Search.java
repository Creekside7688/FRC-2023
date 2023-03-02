// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Search command for automatic balancing.
 * If it does not find the charging system within a set distance then Balancer.java will not be run.
 */

package frc.robot.commands.balancing;

import static frc.robot.Constants.PIDConstants.kD;
import static frc.robot.Constants.PIDConstants.kI;
import static frc.robot.Constants.PIDConstants.kP;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Search extends CommandBase {
    private final DriveTrain driveTrain;

    private PIDController leftPIDController;
    private PIDController rightPIDController;
    private double distance;

    private Encoder lEncoder;
    private Encoder rEncoder;

    public boolean isFinished = false;
    public boolean runBalance = false; // Boolean that tells the program whether or not to run Balancer.java

    public Search(double distance, DriveTrain d) {
        this.distance = distance;
        this.driveTrain = d;

        leftPIDController = new PIDController(kP, kI, kD);
        rightPIDController = new PIDController(kP, kI, kD);

        driveTrain.resetEncoders();

        leftPIDController.setSetpoint(this.distance);
        rightPIDController.setSetpoint(this.distance);

        leftPIDController.setTolerance(5);
        rightPIDController.setTolerance(5);

        this.lEncoder = driveTrain.getLeftEncoder();
        this.rEncoder = driveTrain.getRightEncoder();

        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset booleans.
        isFinished = false;
        runBalance = false;

        driveTrain.resetEncoders();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Driving up onto the charging station increases the pitch.
        if(Math.abs(driveTrain.getPitch()) > 5) {
            runBalance = true;
            isFinished = true;

            driveTrain.resetEncoders(); // This looks useless because it's already in end() but removing this line will break the auto-balancer.
        }

        double lOutput = leftPIDController.calculate(lEncoder.getDistance()*-1);
        double rOutput = rightPIDController.calculate(rEncoder.getDistance()*-1);

        lOutput = MathUtil.clamp(-lOutput, -0.4, 0.4)*-1;
        rOutput = MathUtil.clamp(-rOutput, -0.4, 0.4)*-1;
        driveTrain.tankDrive(lOutput, rOutput);

        SmartDashboard.putNumber("Left Distance", lEncoder.getDistance());
        SmartDashboard.putNumber("Right Distance", rEncoder.getDistance());

        SmartDashboard.putNumber("Left Speed", lOutput);
        SmartDashboard.putNumber("Right Speed", rOutput);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveTrain.Stop();
        driveTrain.resetEncoders();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // End the command if it has found the charging station or if it has reached the
        // end of it's search distance.
        return isFinished || (leftPIDController.atSetpoint() && rightPIDController.atSetpoint());
    }
}
