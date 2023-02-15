// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * Search command for automatic balancing.
 * If it does not find the charging system within a set distance then Balancer.java will not be run.
 */

package frc.robot.commands.balancing;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Search extends CommandBase {
    private final DriveTrain driveTrain;
    private PIDControllerDelegator pidController;

    public boolean isFinished = false;
    public boolean runBalance = false; // Boolean that tells the program whether or not to run Balancer.java

    public Search(DriveTrain d) {
        driveTrain = d;
        pidController = new PIDControllerDelegator(100, driveTrain);

        addRequirements(driveTrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Reset booleans.
        isFinished = false;
        runBalance = false;
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

        double output = pidController.calculate(); // Output does not need to be reversed.
        driveTrain.arcadeDrive(output, 0);

        // Debugging information.
        SmartDashboard.putNumber("Distance :", pidController.getEncoderDistance());
        SmartDashboard.putNumber("Motor Output :", output);
        SmartDashboard.putNumber("Position Error", pidController.getPositionError());
        SmartDashboard.putBoolean("Finished", pidController.atSetpoint());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveTrain.resetEncoders();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // End the command if it has found the charging station or if it has reached the
        // end of it's search distance.
        return isFinished || pidController.atSetpoint();
    }
}
