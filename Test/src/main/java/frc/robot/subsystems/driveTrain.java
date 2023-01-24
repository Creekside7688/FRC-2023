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
  /*
   * private final Encoder rEncoder;
   * private final Encoder lEncoder;
   * 
   */
 
  //1/x 
  private final SlewRateLimiter filter;
  
  public driveTrain() {
    TLFmotor = new WPI_TalonSRX(DriveTrainConstants.TLF_MOTOR);
    TLBmotor = new WPI_VictorSPX(DriveTrainConstants.TLB_MOTOR);
    BLFmotor = new WPI_VictorSPX(DriveTrainConstants.BLF_MOTOR);
    BLBmotor = new WPI_VictorSPX(DriveTrainConstants.BLB_MOTOR);


    TRFmotor = new WPI_TalonSRX(DriveTrainConstants.TRF_MOTOR);
    TRBmotor = new WPI_VictorSPX(DriveTrainConstants.TRB_MOTOR);
    BRFmotor = new WPI_VictorSPX(DriveTrainConstants.BRF_MOTOR);
    BRBmotor = new WPI_VictorSPX(DriveTrainConstants.BRB_MOTOR);
    

    leftmotor = new MotorControllerGroup(TLFmotor,TLBmotor,BLFmotor,BLBmotor);
    rightmotor = new MotorControllerGroup(TRFmotor,TRBmotor,BRBmotor,BRFmotor);
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
