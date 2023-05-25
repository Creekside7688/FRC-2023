// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RGB extends SubsystemBase {
    /** Creates a new RGB. */
    private final Relay redRelay = new Relay(0);
    private final Relay greenRelay = new Relay(1);
    private final Relay blueRelay = new Relay(2);

    public RGB() {
        redRelay.set(Value.kOff);
        greenRelay.set(Value.kOff);
        blueRelay.set(Value.kOff);
    }

    @Override
    public void periodic() {
    }

    public void changeColor(boolean red, boolean green, boolean blue) {
        if(red) {
            redRelay.set(Value.kForward);
        } else {
            redRelay.set(Value.kOff);
        }

        if(green) {
            greenRelay.set(Value.kForward);
        } else {
            greenRelay.set(Value.kOff);
        }

        if(blue) {
            blueRelay.set(Value.kForward);
        } else {
            blueRelay.set(Value.kOff);
        }
    }

    /**
     * @param speed The speed of the robot in centimetres per second.
     */
    public void speedColor(double speed) {
        double newSpeed = Math.abs(speed);
        if(newSpeed == 0) {
            changeColor(true, true, true);
        } else if(newSpeed < 50) {
            changeColor(false, true, false);
        } else if(newSpeed < 100) {
            changeColor(true, true, false);
        } else {
            changeColor(true, false, false);
        }
    }
}
