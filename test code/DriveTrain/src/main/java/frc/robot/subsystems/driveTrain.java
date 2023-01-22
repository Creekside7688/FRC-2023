// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveTrainConstants;


public class driveTrain extends SubsystemBase {
  /** Creates a new driveTrain. */
  // Left motors
  private final WPI_TalonSRX LFmotor;
  private final WPI_VictorSPX LBmotor;
// Right motors
  private final WPI_TalonSRX RFmotor;
  private final WPI_VictorSPX RBmotor;

  private final MotorControllerGroup leftmotor;
  private final MotorControllerGroup rightmotor;

  private final DifferentialDrive diffdrive;
  /*
   * private final Encoder rEncoder;
   * private final Encoder lEncoder;
   * 
   */
 
  //1/x 
  private final SlewRateLimiter filter;
  
  public driveTrain() {
    LFmotor = new WPI_TalonSRX(DriveTrainConstants.KLF_MOTOR);
    LBmotor = new WPI_VictorSPX(DriveTrainConstants.KLB_MOTOR);
    LBmotor.follow(LFmotor);

    RFmotor = new WPI_TalonSRX(DriveTrainConstants.KRF_MOTOR);
    RBmotor = new WPI_VictorSPX(DriveTrainConstants.KRB_MOTOR);
    RBmotor.follow(RFmotor);

    leftmotor = new MotorControllerGroup(LFmotor);
    rightmotor = new MotorControllerGroup(RFmotor);
    leftmotor.setInverted(true);
    rightmotor.setInverted(false);

    filter = new SlewRateLimiter(4);
    diffdrive = new DifferentialDrive(leftmotor, rightmotor);
    /*
     * 
     * rEncoder = new Encoder(DriveTrainConstants.RIGHT_ENCODER[0], DriveTrainConstants.RIGHT_ENCODER[1]);
     * lEncoder = new Encoder(DriveTrainConstants.LEFT_ENCODER[0],DriveTrainConstants.LEFT_ENCODER[1]);
     * rEncoder.setDistancePerPulse(DriveTrainConstants.DISTENCEPERPULS);
     * lEncoder.setDistancePerPulse(DriveTrainConstants.DISTENCEPERPULS);
     * 
     * 
     */
    
    //gyro = new AHRS(SPI.Port.kMXP);


  }

  public void Stop(){
    diffdrive.stopMotor();
  }

  public void arcadeDrive(double speed, double rotation){
    diffdrive.arcadeDrive(filter.calculate(speed*DriveTrainConstants.LIMITSPEED), rotation);

  }
  /*
  
  public void reset_encoders(){
    lEncoder.reset();
    rEncoder.reset();
  }
  
  
  public double getRight_enc_dis(){
    return rEncoder.getDistance();
  }
  public double getLeft_enc_dis(){
    return lEncoder.getDistance();
  }
  public double get_Yaw(){
    return gyro.getAngle();
  }
  public double get_Pitch(){
    return gyro.getPitch();
  }
  public void reset_Yaw(){
    gyro.reset();
  }
  */

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
