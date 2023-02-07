// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;


public class DriveTrain extends SubsystemBase {
  /** Creates a new driveTrain. */
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
  
  public DriveTrain() {
    TLFmotor = new WPI_TalonSRX(DriveTrainConstants.TLF_MOTOR);
    TLBmotor = new WPI_VictorSPX(DriveTrainConstants.TLB_MOTOR);
    BLFmotor = new WPI_VictorSPX(DriveTrainConstants.BLF_MOTOR);
    BLBmotor = new WPI_VictorSPX(DriveTrainConstants.BLB_MOTOR);
    TLFmotor.setNeutralMode(NeutralMode.Coast);
    TLBmotor.setNeutralMode(NeutralMode.Coast);
    BLFmotor.setNeutralMode(NeutralMode.Coast);
    BLBmotor.setNeutralMode(NeutralMode.Coast);


    TRFmotor = new WPI_TalonSRX(DriveTrainConstants.TRF_MOTOR);
    TRBmotor = new WPI_VictorSPX(DriveTrainConstants.TRB_MOTOR);
    BRFmotor = new WPI_VictorSPX(DriveTrainConstants.BRF_MOTOR);
    BRBmotor = new WPI_VictorSPX(DriveTrainConstants.BRB_MOTOR);
    TRFmotor.setNeutralMode(NeutralMode.Coast);
    TRBmotor.setNeutralMode(NeutralMode.Coast);
    BRFmotor.setNeutralMode(NeutralMode.Coast);
    BRBmotor.setNeutralMode(NeutralMode.Coast);
    

    leftmotor = new MotorControllerGroup(TLFmotor,TLBmotor,BLFmotor,BLBmotor);
    rightmotor = new MotorControllerGroup(TRFmotor,TRBmotor,BRBmotor,BRFmotor);
    leftmotor.setInverted(true);
    rightmotor.setInverted(false);

    filter = new SlewRateLimiter(2);
    diffdrive = new DifferentialDrive(leftmotor, rightmotor);

    rEncoder = new Encoder(DriveTrainConstants.RIGHT_ENCODER[0], DriveTrainConstants.RIGHT_ENCODER[1], false);
    lEncoder = new Encoder(DriveTrainConstants.LEFT_ENCODER[0],DriveTrainConstants.LEFT_ENCODER[1], true);
    lEncoder.setDistancePerPulse(DriveTrainConstants.DISTENCEPERPULS);
    rEncoder.setDistancePerPulse(DriveTrainConstants.DISTENCEPERPULS);

    gyro = new AHRS(Port.kUSB1);

    this.reset();
  }

  public void Stop(){
    diffdrive.stopMotor();
  }

  public void arcadeDrive(double speed, double rotation){
    diffdrive.arcadeDrive(filter.calculate(speed * DriveTrainConstants.LIMITSPEED), rotation);
  }
  
  //public double getYaw() {
  //   return gyro.getAngle();
  //}
  // public double getPitch(){
  //   return gyro.getPitch();
  // }
  // public double getRoll() {
  //   return gyro.getRoll();
  // }

  public double[] getEncoders() {
    return new double[] {lEncoder.getDistance(), rEncoder.getDistance() };
  }

  public double getEncoderAverage() {
    return (lEncoder.getDistance() + rEncoder.getDistance()) / 2;
  }

  // public void calibrateGyro() {
  //   gyro.calibrate();
  // }

  public void reset() {
    lEncoder.reset();
    rEncoder.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
