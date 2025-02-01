// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ejml.equation.Variable;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
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
  DifferentialDriveKinematics kinematics;
  DifferentialDriveWheelSpeeds wheelSpeeds;
  DifferentialDriveOdometry odometry;
  ChassisSpeeds chassisSpeeds;
  double leftVelocity;
  double rightVelocity;
  AnalogGyro gyro;

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

    kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.TRACKWIDTH);
    chassisSpeeds = new ChassisSpeeds();
    wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    leftVelocity = wheelSpeeds.leftMetersPerSecond;
    rightVelocity = wheelSpeeds.rightMetersPerSecond;

    leftEncoder = new Encoder(0,1);
    rightEncoder = new Encoder(2, 3);
    leftEncoder.reset();
    rightEncoder.reset();
    leftEncoder.setDistancePerPulse(2 * Math.PI * Constants.DriveConstants.WHEELRADIUS / Constants.DriveConstants.ENCODERRESOLUTION);
    rightEncoder.setDistancePerPulse(2 * Math.PI * Constants.DriveConstants.WHEELRADIUS / Constants.DriveConstants.ENCODERRESOLUTION);
    gyro = new AnalogGyro(0);
    gyro.reset();
    odometry = new DifferentialDriveOdometry
    (gyro.getRotation2d(), rightEncoder.getDistance(), leftEncoder.getDistance(), new Pose2d(0,0, new Rotation2d()));
    drive = new DifferentialDrive(leadLeftMotor, leadRightMotor);
  }

  // public Command moveForward() {
  //   return run(() -> {
  //     leadLeftMotor.set(Constants.DriveConstants.DRIVEMOVESPEED);
  //     leadRightMotor.set(Constants.DriveConstants.DRIVEMOVESPEED);
  //   });
  // }
  // public Command moveBackward() {
  //   return run(() -> {
  //     leadLeftMotor.set(Constants.DriveConstants.DRIVEREVERSESPEED);
  //     leadRightMotor.set(Constants.DriveConstants.DRIVEREVERSESPEED);
  //   });
  // }
  // public Command rotateLeft() {
  //   return run(() -> {
  //     leadLeftMotor.set(Constants.DriveConstants.DRIVEREVERSESPEED);
  //     leadRightMotor.set(Constants.DriveConstants.DRIVEMOVESPEED);
  //   });
  // }
  // public Command rotateRight() {
  //   return run(() -> {
  //     leadLeftMotor.set(Constants.DriveConstants.DRIVEMOVESPEED);
  //     leadRightMotor.set(Constants.DriveConstants.DRIVEREVERSESPEED);
  //   });
  // }

   // sets the speed of the drive motors
  public void driveArcade(double xSpeed, double zRotation) {
    drive.arcadeDrive(xSpeed, zRotation);
  }
  
  @Override
  public void periodic() {
    // odometry.update(gyro.getRotation2d(), null);
    var gryoAngle = gyro.getRotation2d();
    var pose = odometry.update(gryoAngle,
    leftEncoder.getDistance(),
    rightEncoder.getDistance());
  }
}
