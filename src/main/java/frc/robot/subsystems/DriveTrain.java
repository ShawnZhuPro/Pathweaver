// Based upon 2021's Competition Season DriveTrain code

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.kauailabs.navx.frc.AHRS;

// test
public class DriveTrain extends SubsystemBase 
{
  private final WPI_TalonSRX leftDriveTalon;
  private final WPI_TalonSRX rightDriveTalon;

  private AHRS navx = new AHRS(SPI.Port.kMXP);

  private ShuffleboardTab DTTab = Shuffleboard.getTab("DriveTrain");
  private GenericEntry LeftVoltage = DTTab.add("Left Voltage", 0.0).getEntry();
  private GenericEntry RightVoltage = DTTab.add("Right Voltage", 0.0).getEntry();

  /** Creates a new DriveTrain */
  public DriveTrain() 
  {
    leftDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.LeftDriveTalonPort);
    rightDriveTalon = new WPI_TalonSRX(Constants.DriveTrainPorts.RightDriveTalonPort);
  
    leftDriveTalon.setNeutralMode(NeutralMode.Coast);
    rightDriveTalon.setNeutralMode(NeutralMode.Coast);

    leftDriveTalon.setInverted(true);
    rightDriveTalon.setInverted(false);

    leftDriveTalon.setSensorPhase(true);
    rightDriveTalon.setSensorPhase(true);

    leftDriveTalon.configFactoryDefault();
    leftDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    rightDriveTalon.configFactoryDefault();
    rightDriveTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

  }

  public void tankDrive(double leftSpeed, double rightSpeed) {
    rightDriveTalon.set(rightSpeed);
    leftDriveTalon.set(leftSpeed);
  }

  public void resetEncoders() {
    leftDriveTalon.setSelectedSensorPosition(0,0,10);
    rightDriveTalon.setSelectedSensorPosition(0,0,10);
  }

  public double getTicks() {
    return (leftDriveTalon.getSelectedSensorPosition(0) + rightDriveTalon.getSelectedSensorPosition(0)) / 2.0;
  }

  public double getDisplacement(){
    // (Diameter in meters of the wheel * pi / 4096 (this is the number of ticks per wheel revolution)) * getTicks();
    return 0;
  }

  /**
   * Retrieves the robot's current heading in degrees and adjusts it for the standard counterclockwise measurement convention.
   *
   * @return The current orientation or heading of the robot in degrees.
   */
  public double getHeading() {
    // The 'navx' object likely represents a sensor providing orientation data.
    // 'getRotation2d()' retrieves the current 2D rotation information from the sensor.
    // 'getDegrees()' obtains the rotation angle in degrees from the 2D rotation data.
    // The negative sign is applied to ensure the angle is measured counterclockwise from a reference direction,
    // aligning with the standard convention used in mathematics and robotics.
    return -navx.getRotation2d().getDegrees();
  }

  public void zeroHeading() {
    navx.reset();
  }
 
  public double getAngle(){
    return navx.getAngle(); 
  }
 
  public void resetNavx(){
    navx.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Voltage", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Right Voltage", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("Angle", navx.getAngle());
    SmartDashboard.putNumber("Displacement(meters)", getDisplacement());

    LeftVoltage.setDouble(leftDriveTalon.getMotorOutputPercent());
    RightVoltage.setDouble(rightDriveTalon.getMotorOutputPercent());

  }
}