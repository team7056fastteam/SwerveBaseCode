package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class Robot extends TimedRobot {
  SwerveSubsystem _drive;

  //plug your gyro rotation value into this and you will get field orientation if not then you will have robot orientation
  double gyroRotation = 0.0;

  //driver controllers I am using xbox ones you will probally need to import a different one
  XboxController driver = new XboxController(0);
  XboxController operator = new XboxController(1);
  //these control the speed of the robot forward/backward sideways and rotations
  double driveX;
  double driveY;
  double driveZ;

  //these add a smoother to reduce stickstabbing
  SlewRateLimiter xLimiter, yLimiter, zLimiter;

  //array for the state of the modules
  SwerveModuleState[] moduleStates;

  //The values the wheels get set to when locking them in a X
  
  SwerveModuleState[] lockedStates = 
  {
    new SwerveModuleState(0, new Rotation2d(Math.toRadians(45))),
    new SwerveModuleState(0, new Rotation2d(Math.toRadians(315))),
    new SwerveModuleState(0, new Rotation2d(Math.toRadians(315))),
    new SwerveModuleState(0, new Rotation2d(Math.toRadians(45)))
  };
  
  ChassisSpeeds targetChassisSpeeds;

  //when enabled will lock the chassis during auto
  boolean lockAuton = false;

  @Override
  public void robotInit() {
    _drive = new SwerveSubsystem();

    //smoothers taking in the acceleration constant
    xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
    zLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //sets the x,y,z speeds
    moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);

    //this function is what locks the wheels
    if(!lockAuton){ _drive.setModuleStates(moduleStates); } else{ _drive.setModuleStatesUnrestricted(lockedStates); }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //Driver Functions
    //Gets the drivers joysticks values
    driveX = (driver.getRawAxis(1) * -1);
    driveY = (driver.getRawAxis(0) * -1);
    driveZ = (driver.getRawAxis(4) * -1);
    //apply deadband
    driveX = Math.abs(driveX) > DriveConstants.kDeadband ? driveX : 0.0;
    driveY = Math.abs(driveY) > DriveConstants.kDeadband ? driveY : 0.0;
    driveZ = Math.abs(driveZ) > DriveConstants.kDeadband ? driveZ : 0.0;
    //smoother
    driveX = xLimiter.calculate(driveX) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    driveY = yLimiter.calculate(driveY) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
    driveZ = zLimiter.calculate(driveZ) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
    targetChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(driveX, driveY, driveZ, getGyroscopeRotation2d());
    
    if (driver.getRawAxis(2) > 0.1) {
      //lock
      _drive.setModuleStatesUnrestricted(lockedStates);
    }
    else if(driver.getRightBumper()){
      //Robot Centric
      ChassisSpeeds modifiedChassisSpeeds =  new ChassisSpeeds(driveX, driveY, driveZ);
      moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(modifiedChassisSpeeds);
      _drive.setModuleStates(moduleStates);
    }
    else{
      //field Centric
      moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetChassisSpeeds);
      _drive.setModuleStates(moduleStates);
    }
  }
  //gets a rot2d from the gyroRotation variable
  public Rotation2d getGyroscopeRotation2d() {
      return Rotation2d.fromDegrees(gyroRotation);
  }

  //stops the wheels from moving
  public void stop() {
    _drive.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates
    (ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0, getGyroscopeRotation2d())));
  }

  @Override
  public void disabledInit() {
    stop();
  }
}
