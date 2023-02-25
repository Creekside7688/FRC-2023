// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveTrainConstants.*;

public class DriveTrain extends SubsystemBase {

    // Left motors
    private final WPI_TalonSRX TLFmotor;
    private final WPI_VictorSPX TLBmotor;
    private final WPI_VictorSPX BLFmotor;
    private final WPI_VictorSPX BLBmotor;

    // Right motors
    private final WPI_TalonSRX TRFmotor;
    private final WPI_VictorSPX TRBmotor;
    private final WPI_VictorSPX BRBmotor;
    private final WPI_VictorSPX BRFmotor;

    private final MotorControllerGroup leftmotor;
    private final MotorControllerGroup rightmotor;

    private final DifferentialDrive diffdrive;

    private final AHRS gyro;

    private final Encoder lEncoder;
    private final Encoder rEncoder;

    private final SlewRateLimiter filter;

    private final NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public DriveTrain() {
        TLFmotor = new WPI_TalonSRX(TLF_MOTOR);
        TLBmotor = new WPI_VictorSPX(TLB_MOTOR);
        BLFmotor = new WPI_VictorSPX(BLF_MOTOR);
        BLBmotor = new WPI_VictorSPX(BLB_MOTOR);
        TLFmotor.setNeutralMode(NeutralMode.Coast);
        TLBmotor.setNeutralMode(NeutralMode.Coast);
        BLFmotor.setNeutralMode(NeutralMode.Coast);
        BLBmotor.setNeutralMode(NeutralMode.Coast);

        TRFmotor = new WPI_TalonSRX(TRF_MOTOR);
        TRBmotor = new WPI_VictorSPX(TRB_MOTOR);
        BRFmotor = new WPI_VictorSPX(BRF_MOTOR);
        BRBmotor = new WPI_VictorSPX(BRB_MOTOR);
        TRFmotor.setNeutralMode(NeutralMode.Coast);
        TRBmotor.setNeutralMode(NeutralMode.Coast);
        BRFmotor.setNeutralMode(NeutralMode.Coast);
        BRBmotor.setNeutralMode(NeutralMode.Coast);

        leftmotor = new MotorControllerGroup(TLFmotor, TLBmotor, BLFmotor, BLBmotor);
        rightmotor = new MotorControllerGroup(TRFmotor, TRBmotor, BRBmotor, BRFmotor);

        leftmotor.setInverted(true);
        rightmotor.setInverted(false);

        filter = new SlewRateLimiter(2);
        diffdrive = new DifferentialDrive(leftmotor, rightmotor);

        lEncoder = new Encoder(LEFT_ENCODER[0], LEFT_ENCODER[1], true);
        rEncoder = new Encoder(RIGHT_ENCODER[0], RIGHT_ENCODER[1], false);

        lEncoder.setDistancePerPulse(DISTANCE_PER_PULS);
        rEncoder.setDistancePerPulse(DISTANCE_PER_PULS);

        gyro = new AHRS(Port.kUSB1);

        this.resetEncoders();
        gyro.calibrate();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pitch", gyro.getPitch());
        SmartDashboard.putNumber("Yaw", gyro.getYaw());
        SmartDashboard.putNumber("Roll", gyro.getRoll());
    }

    public void Stop() {
        diffdrive.stopMotor();
    }

    public void arcadeDrive(double speed, double rotation) {
        diffdrive.arcadeDrive(filter.calculate(speed * LIMIT_SPEED), rotation);
    }

    public void tankDrive(double leftSpeed, double rightSpeed) {
        diffdrive.tankDrive(leftSpeed, rightSpeed);
    }

    public double getYaw() {
        return gyro.getAngle();
    }

    public double getPitch() {
        return gyro.getPitch();
    }

    public double getRoll() {
        return gyro.getRoll();
    }

    public Encoder getLeftEncoder() {
        return lEncoder;
    }

    public Encoder getRightEncoder() {
        return rEncoder;
    }

    public double getEncoderAverage() {
        return (lEncoder.getDistance() + rEncoder.getDistance()) / 2;
    }

    public void resetEncoders() {
        lEncoder.reset();
        rEncoder.reset();
    }

    public void setCoast() {
        TLFmotor.setNeutralMode(NeutralMode.Coast);
        TLBmotor.setNeutralMode(NeutralMode.Coast);
        BLFmotor.setNeutralMode(NeutralMode.Coast);
        BLBmotor.setNeutralMode(NeutralMode.Coast);

        TRFmotor.setNeutralMode(NeutralMode.Coast);
        TRBmotor.setNeutralMode(NeutralMode.Coast);
        BRFmotor.setNeutralMode(NeutralMode.Coast);
        BRBmotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setBrake() {
        TLFmotor.setNeutralMode(NeutralMode.Brake);
        TLBmotor.setNeutralMode(NeutralMode.Brake);
        BLFmotor.setNeutralMode(NeutralMode.Brake);
        BLBmotor.setNeutralMode(NeutralMode.Brake);
        TRFmotor.setNeutralMode(NeutralMode.Brake);
        TRBmotor.setNeutralMode(NeutralMode.Brake);
        BRFmotor.setNeutralMode(NeutralMode.Brake);
        BRBmotor.setNeutralMode(NeutralMode.Brake);
    }

    public double getXLimelight() {
        NetworkTableEntry tx = table.getEntry("tx");
        return tx.getDouble(0);
    }

    public double getValidTarget() {
        NetworkTableEntry tv = table.getEntry("tv");
        return tv.getDouble(0);
    }

    public double getTargetArea() {
        NetworkTableEntry ta = table.getEntry("ta");
        return ta.getDouble(0);
    }
}
