// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import poplib.src.main.java.poplib.sensors.gyro.Gyro;

public class Drive extends SubsystemBase {
  SparkMax leadRightMotor;
  SparkMax followRightMotor;
  SparkMax leadLeftMotor;
  SparkMax followLeftMotor;
  private Encoder leftEncoder;
  private Encoder rightEncoder;
  private final DifferentialDrive drive;
  // SwerveDriveOdometry odometry;
  // SwerveDriveKinematics kinematics;
  // AnalogGyro gyro;
  // Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
  // Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
  // Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
  // Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

  private static Drive instance;

  public static Drive getInstance() {
      if (instance == null) {
          instance = new Drive();
        }
      return instance;
  }

  public Drive() {
    leadLeftMotor = Constants.DriveConstants.LeadLeftMotor.createSparkMax();
    followLeftMotor = Constants.DriveConstants.FollowLeftMotor.createSparkMax();
    leadRightMotor = Constants.DriveConstants.LeadRightMotor.createSparkMax();
    followRightMotor = Constants.DriveConstants.FollowRightMotor.createSparkMax();
    // leftEncoder = new Encoder(1,2);
    // rightEncoder = new Encoder(3,4);
    // odometry = new SwerveDriveOdometry(null, gyro.getRotation2d(), null, null);
    // kinematics = new SwerveDriveKinematics(null);
    drive = new DifferentialDrive(leadLeftMotor, leadRightMotor);
    // gyro = new AnalogGyro(1);
  }

  public Command moveForward() {
    return run(() -> {
      leadLeftMotor.set(Constants.DriveConstants.DRIVEMOVESPEED);
      leadRightMotor.set(Constants.DriveConstants.DRIVEMOVESPEED);
    });
  }

  public Command moveBackward() {
    return run(() -> {
      leadLeftMotor.set(Constants.DriveConstants.DRIVEREVERSESPEED);
      leadRightMotor.set(Constants.DriveConstants.DRIVEREVERSESPEED);
    });
  }

  public Command rotateLeft() {
    return run(() -> {
      leadLeftMotor.set(Constants.DriveConstants.DRIVEREVERSESPEED);
      leadRightMotor.set(Constants.DriveConstants.DRIVEMOVESPEED);
    });
  }

  public Command rotateRight() {
    return run(() -> {
      leadLeftMotor.set(Constants.DriveConstants.DRIVEMOVESPEED);
      leadRightMotor.set(Constants.DriveConstants.DRIVEREVERSESPEED);
    });
  }

   // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
  
  @Override
  public void periodic() {
    // odometry.update(gyro.getRotation2d(), null);
  }
}
