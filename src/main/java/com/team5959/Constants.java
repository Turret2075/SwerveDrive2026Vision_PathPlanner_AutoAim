package com.team5959;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

//  =================
//       Red CAN
//  =================
//   0 · roboRIO
//   1 · PDH

//   2 · frontLeftRotation
//   3 · frontLeftDrive
//   4 · frontRightRotation
//   5 · frontRightDrive
//   6 · rearRightRotation   //FIXME check CAN ID's out
//   7 · rearRightDrive
//   8 · rearLeftRotation
//   9 · rearLeftDrive

//   16 · elevatorRight  70:1
//   17 · elevatorLeft

//   18 · armIntakeMotor  90:1
//   19 · algaeIntakeMotor 30:1

//   20 · coralIntakeMotorRight 25:1
//   21 · coralIntakeMotorLeft 25:1

//   22 · miniArmMotor 50:1
//  =================
//30 CanRange

public class Constants {
  
  //Boolean que activa simulacion de MapleSim
  public static final boolean ShouldKillRoboRio = true;    


    public static class ControllerConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
      }
    
      public static class SwerveConstants {
        //SDS L2 
        public static final boolean ROTATION_ENCODER_DIRECTION = false; 
    
        /* * * MEASUREMENTS * * */
        public static final double WHEEL_DIAMETER = 4 * 2.54 / 100; //Diametro en metros
        public static final double TRACK_WIDTH = 0.5200;
        public static final double WHEEL_BASE = 0.5200;
      
        public static final double DRIVE_GEAR_RATIO = 6.75 / 1;
        public static final double ROTATION_GEAR_RATIO = 150 / 7;
        
        public static final double VOLTAGE = 12;
    
        /* * * SWERVE DRIVE KINEMATICS * * */
        // ORDER IS ALWAYS FL, BL, FR, BR 
        //pos x is positive out in front, pos y is positive to the left 
        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
          
          // front left
          new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
          // back left
          new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
          // front right
          new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
          // back right
          new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)                 
    
        );
    
        /* * * FRONT LEFT * * */
        
        public static class FrontLeft {
          public static final int DRIVE_PORT = 3;
          public static final int ROTATION_PORT = 2;
          public static final int ABSOLUTE_ENCODER_PORT = 10;
          public static final double OFFSET = (0.39209); //80.95; (-0.3986 * 90)//este ya está bien-143.87
          public static final boolean DRIVE_INVERTED = false; 
          public static final boolean ROTATION_INVERTED = true; 
          public static final boolean CANCODER_INVERTED = false;
    
          public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED, CANCODER_INVERTED);
        }
    
        /* * * BACK LEFT * * */
       
        public static class BackLeft {
          public static final int DRIVE_PORT = 9;
          public static final int ROTATION_PORT = 8;
          public static final int ABSOLUTE_ENCODER_PORT = 13;
          public static final double OFFSET = (0.10351); //(-0.0927 * 90)-101.60 + 6;
          public static final boolean DRIVE_INVERTED = false; 
          public static final boolean ROTATION_INVERTED = true; 
          public static final boolean CANCODER_INVERTED = false;
    
          public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED, CANCODER_INVERTED);
        }

        /* * * FRONT RIGHT * * */
          public static class FrontRight {
          public static final int DRIVE_PORT = 5;
          public static final int ROTATION_PORT = 4;
          public static final int ABSOLUTE_ENCODER_PORT = 11;
          public static final double OFFSET = (-0.43774); //-25.31 - 2;(0.4321 * 55)
          public static final boolean DRIVE_INVERTED = true; 
          public static final boolean ROTATION_INVERTED = true; 
          public static final boolean CANCODER_INVERTED = false;
    
          public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED, CANCODER_INVERTED);
        }
    
        /* * * BACK RIGHT * * */
        // FILL IN VALUES FOR BACK RIGHT 
        public static class BackRight {
          public static final int DRIVE_PORT = 7;
          public static final int ROTATION_PORT = 6;
          public static final int ABSOLUTE_ENCODER_PORT = 12;
          public static final double OFFSET = (0.20160); //(-0.2290 * )-28.92 + 6;
          public static final boolean DRIVE_INVERTED = true; 
          public static final boolean ROTATION_INVERTED = true; 
          public static final boolean CANCODER_INVERTED = false;
    
          public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_PORT, ROTATION_PORT, ABSOLUTE_ENCODER_PORT, OFFSET, DRIVE_INVERTED, ROTATION_INVERTED, CANCODER_INVERTED);
        }
        
    
        
        
        /* * * CONVERSIONS FOR ENCODERS * * */
        //velocity in meters per sec instead of RPM 
        public static final double DRIVE_ENCODER_POSITION_CONVERSION = ((2 * Math.PI * (WHEEL_DIAMETER/2))) / DRIVE_GEAR_RATIO; //drive enc rotation
        //velocity in meters instead of rotations 
        public static final double DRIVE_ENCODER_VELOCITY_CONVERSION = DRIVE_ENCODER_POSITION_CONVERSION / 60; //drive enc speed por segundo.
     
        /* * * PID VALUES FOR TURNING MOTOR PID * * */
        public static final double KP_TURNING = 0.006;
        public static final double KI_TURNING = 0.0002;
        public static final double KD_TURNING = 0.0001;

        public static final double DRIVE_KP = 0.001;
        public static final double DRIVE_KI = 0.0;
        public static final double DRIVE_KD = 0.0001;

        public static final double DRIVE_KS = 0.07;
        public static final double DRIVE_KV = 3.3;
        public static final double DRIVE_KA =0.2;




    
        public static final double KP_AUTO_XController = 2.9;
        public static final double KI_AUTO_XController = 0.0005;
        public static final double KD_AUTO_XController = 0.001;
        public static final double AUTO_XTOLLERANCE = 0.025; // tolerance in meters

        public static final double KP_AUTO_YController = 2.85;
        public static final double KI_AUTO_YController = 0.0005;
        public static final double KD_AUTO_YController = 0.001;
        public static final double AUTO_YTOLLERANCE = 0.025; // tolerance in meters
    
        public static final double KP_AUTO_ROTATION = 1.1;
        public static final double KI_AUTO_ROTATION = 0.0005;
        public static final double KD_AUTO_ROTATION = 0.001;
        public static final double ROTATION_TOLLERANCE = 1; // tolerance in dergrees

        public static final double KP_AUTO_HOLDING = 0.15;
        public static final double KI_AUTO_HOLDING = 0.000;
        public static final double KD_AUTO_HOLDING = 0.001;
        public static final double HOLDING_TOLLERANCE = 0.1; // tolerance in dergrees

        public static final double KP_TRANS_PATHPLANNER =5;
        public static final double KI_TRANS_PATHPLANNER = 0.0001;
        public static final double KD_TRANS_PATHPLANNER = 0.001;
        public static final double PIDPATHPLANNER_TRANS_TOLLERANCE = 1; // tolerance in dergrees

        public static final double KP_ROT_PATHPLANNER = 5;
        public static final double KI_ROT_PATHPLANNER = 0.001;
        public static final double KD_ROT_PATHPLANNER = 0.0001;
        public static final double PIDPATHPLANNER_ROT_TOLLERANCE = 1; // tolerance in dergrees
    
    
        /* * * MAX * * */
        public static final double MAX_SPEED = 3.2; //meters per second
        public static final double MAX_ROTATION = MAX_SPEED / Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);


        //AUTONOMOUS CONSTANTS
        public static final double MAX_AUTO_SPEED = 2; //meters per second
        public static final double MAX_AUTO_ACCELERATION = 1.5; //meters per second squared

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                  Math.toRadians(180), Math.toRadians(360));

            
      }
    }
    

