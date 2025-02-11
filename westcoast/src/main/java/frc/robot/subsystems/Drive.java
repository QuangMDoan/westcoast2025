// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import poplib.src.main.java.poplib.control.PIDConfig;
import poplib.src.main.java.poplib.sensors.gyro.Gyro;
import poplib.src.main.java.poplib.smart_dashboard.PIDTuning;
import poplib.src.main.java.poplib.swerve.swerve_templates.BaseSwerve;

public class Drive extends SubsystemBase {
  private SparkMax leadRightMotor;
  private SparkMax followRightMotor;
  private SparkMax leadLeftMotor;
  private SparkMax followLeftMotor;
  private final DifferentialDrive drive;
  private DifferentialDriveKinematics kinematics;
  private DifferentialDriveOdometry odometry;
  private AHRS gyro;
  private static PIDController xpid;
  private static PIDController thetapid;
  private static double turnSetpoint;
  // private static PIDTuning angleTuning;
  // private static PIDTuning driveTuning;

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
    
    kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH);
    
   
    gyro = new AHRS(null);
    gyro.reset();
    odometry = new DifferentialDriveOdometry
    (gyro.getRotation2d(), leadLeftMotor.getEncoder().getPosition(), 
    leadRightMotor.getEncoder().getPosition(), new Pose2d(0,0, new Rotation2d()));

    xpid = Constants.DriveConstants.X_PID_CONFIG.getPIDController(); 
    thetapid = Constants.DriveConstants.THETA_PID_CONFIG.getPIDController();

    turnSetpoint = 0.0;
    thetapid.setSetpoint(turnSetpoint);

    drive = new DifferentialDrive(leadLeftMotor, leadRightMotor);
  }

  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(), leadLeftMotor.getEncoder().getPosition(), leadRightMotor.getEncoder().getPosition());
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

  // field oriented
  public Command turn(double angle){
    return run(() -> {
      leadLeftMotor.set(thetapid.calculate(gyro.getAngle())*Constants.DriveConstants.DRIVE_MOVE_SPEED);
    }).alongWith(run(() -> {
      leadRightMotor.set(thetapid.calculate(gyro.getAngle())*Constants.DriveConstants.DRIVE_REVERSE_SPEED);
    }));
  }

  public Command pleaseDrive(double velocity){
    return run(() -> {
      leadLeftMotor.set(xpid.calculate(velocity));
    }).alongWith(run(() -> {
      leadRightMotor.set(xpid.calculate(velocity));
    }));
  }

  // public void driveArcade(double xSpeed, double zRotation) {
  //   drive.arcadeDrive(xSpeed, zRotation);
  // }

  public void drive(Translation2d vector, double rotation) {

  }
  
  @Override
  public void periodic() {
    updateOdometry();

  }
}
