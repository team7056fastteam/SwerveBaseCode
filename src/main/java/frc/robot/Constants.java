package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final class ModuleConstants {
        //wheel diameter in inches
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);

        //look these numbers up
        public static final double kDriveMotorGearRatio = 1 / 5.8462;
        public static final double kTurningMotorGearRatio = 1 / 18;

        //these convert thoose numbers into positon and velocity
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    }
    public static final class DriveConstants {
        //measure left to right wheel
        public static final double kTrackWidth = 0.5;

        //measure front to back wheel
        public static final double kWheelBase = 0.5;

        //should be same if you have a square robot which is typically what you want

        public static SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kTrackWidth / 2.0, kWheelBase / 2.0),
            new Translation2d(kTrackWidth / 2.0, -kWheelBase / 2.0),
            new Translation2d(-kTrackWidth / 2.0, kWheelBase / 2.0),
            new Translation2d(-kTrackWidth / 2.0, -kWheelBase / 2.0));

        //drive motors port ids
        public static final int kFrontLeftDriveMotorPort = 0;
        public static final int kBackLeftDriveMotorPort = 1;
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kBackRightDriveMotorPort = 3;

        //turn motors port ids
        public static final int kFrontLeftTurningMotorPort = 0;
        public static final int kBackLeftTurningMotorPort = 1;
        public static final int kFrontRightTurningMotorPort = 2;
        public static final int kBackRightTurningMotorPort = 3;

        //if the wheels are turning forever they flip the corresponding value
        public static final boolean kFrontLeftTurningMotorReversed = false;
        public static final boolean kBackLeftTurningMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = false;
        public static final boolean kBackRightTurningMotorReversed = false;

        //to test this put robot up so wheels aren't touching ground and if you put the stick all the way forward they should all be driving in the forward direction
        //if not they adjust this value
        public static final boolean kFrontLeftDriveMotorReversed = true;
        public static final boolean kBackLeftDriveMotorReversed = false;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kBackRightDriveMotorReversed = true;

        //abs encoders ids
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 3;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 4;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 5;
        public static final int kBackRightDriveAbsoluteEncoderPort = 2;

        //don't think this needs to be adjusted
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        //adjust wheel offsets
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = Math.toRadians(0);

        //these are the physical max of the motor. Look up the values for these.
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        //adjust the divisor closer to 1 but never past if you want more speed
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1.5;

        //adjust the divisor closer to 1 but never past if you want faster turning
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 3.5;

        //adjust these values for faster acceleration during teleOp
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 1.5;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 1.5;

        //adjust this value if your robot is moving without you touching the sticks. the older controller the more this number typically is
        //you probally want to replace controllers after two seasons or if the stick drift is too high for preicous movement of robot
        public static final double kDeadband = 0.07;
    }
}
