package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.util.Units;


public class Constants {

    public static final class IntakeConstants {
        public static final int intakeLeft = 2;
        public static final int intakeRight = 1;
        public static final int intakeChooser = 8;
        public static final int shooterFeeder = 7;
      }


    public static final class ElevatorConstants {

        public static final int elevator = 10;
        public static final int shooter = 9;
        public static double kGearRatio = 9.0;
        public static final double kMeterPerRevolution = Units.inchesToMeters((1.888*Math.PI)/kGearRatio);

        public static final double KP = 3.0;
        public static final double KI = 0.000000;
        public static final double KD = 0.0000;
        public static final double KIz = 0.0;
        public static final double KFF = 0.3;
        public static final double MaxOutput = 1.0;
        public static final double kMinOutput = -1.0;
        public static final double MaxRPM = 5700;
        // create a max amps so we do not destroy the motors


        public static final double elevatorSpeed = 0.005;
        public static final double maxHeight = .1;
        public static final IdleMode kIdleMode = IdleMode.kBrake;

        public static final int kFeederStation = 0;
        public static final int kLevel1 = 0;
        public static final int kLevel2 = 0;
        public static final int kLevel3 = 1;
        public static final int kLevel4 = 150;
          

    }

    public static final class OIConstants {
        public static final int kDriverOneControllerPort = 0;
        public static final int kDriverTwoControllerPort = 1;
        public static final double kDriveDeadband = 0.1;
        public static final double kTriggerButtonThreshold = 0.2;
      }
}