// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.team5959.Constants.SwerveConstants;
import com.team5959.subsystems.SwerveChassis;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MapleSwerve extends SubsystemBase{
  /** Creates a new MapleSwerve. */
  private final SelfControlledSwerveDriveSimulation simulatedChassis;
  private final Field2d simField2d;
  private final PIDController headingPID = new PIDController(SwerveConstants.KP_AUTO_HOLDING,SwerveConstants.KI_AUTO_HOLDING, SwerveConstants.KD_AUTO_HOLDING); // TUNEAR
  private double headingSetpointDeg = 0.0;
  private boolean headingHoldEnabled = false;

  public MapleSwerve() {

    headingPID.enableContinuousInput(-180.0, 180.0);
    headingPID.setTolerance(SwerveConstants.HOLDING_TOLLERANCE); // TUNEAR
    headingPID.setIZone(5);

    final DriveTrainSimulationConfig config = DriveTrainSimulationConfig.
      Default().
      withGyro(COTS.ofNav2X()).
      withSwerveModule(COTS.ofMark4i(
        DCMotor.getNEO(1),
        DCMotor.getNEO(1),
        COTS.WHEELS.COLSONS.cof, 
        2)).
      withTrackLengthTrackWidth(Inches.of(26), Inches.of(26)).
      withBumperSize(Inches.of(32), Inches.of(32)
    );

    this.simulatedChassis = new SelfControlledSwerveDriveSimulation(
      new SwerveDriveSimulation(config, new Pose2d(0, 0, new Rotation2d())));
    
    SimulatedArena.getInstance().addDriveTrainSimulation(simulatedChassis.getDriveTrainSimulation());

    RobotConfig configPathPlanner = null;
    try{
      configPathPlanner = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
      this::getPose, 
      this::setPose, 
      this::getRobotRelativeSpeeds, 
      this::driveRobotRelative, 
      new PPHolonomicDriveController(
        new PIDConstants(SwerveConstants.KP_TRANS_PATHPLANNER, SwerveConstants.KI_TRANS_PATHPLANNER, SwerveConstants.KD_TRANS_PATHPLANNER),
        new PIDConstants(SwerveConstants.KP_ROT_PATHPLANNER, SwerveConstants.KI_ROT_PATHPLANNER, SwerveConstants.KD_ROT_PATHPLANNER)), 
      configPathPlanner, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      this);

    simField2d = new Field2d();
    SmartDashboard.putData("Simulated Field", simField2d);

  }

  public void driveMaple(Translation2d translation, double rotation, boolean fieldOriented, boolean isOpenLoop){
    this.simulatedChassis.runChassisSpeeds(
      new ChassisSpeeds(translation.getX(), translation.getY(), rotation), 
      new Translation2d(), 
      fieldOriented, 
      true);
  }

    //DRIVE
  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented){
    SwerveModuleState[] states;
    if (fieldOriented) {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d())

      );
    } else {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
      );
    }

    setModuleStates(states);

  }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
      simulatedChassis.runSwerveStates(desiredStates);
    }

    //STOP 
  public void stopModules() {
    simulatedChassis.runChassisSpeeds(
      new ChassisSpeeds(), 
      new Translation2d(), 
      false, 
      false);
    }

    public ChassisSpeeds getMeasuredSpeeds() {
      return simulatedChassis.getMeasuredSpeedsFieldRelative(true);
    }

    public Rotation2d getRotation2d() {
      return simulatedChassis.getRawGyroAngle();
    }


    public Pose2d getPose() {
      return simulatedChassis.getOdometryEstimatedPose();
    }

    public void holdCurrentHeading() {
      headingSetpointDeg = getRotation2d().getDegrees();
      headingHoldEnabled = true;
    }

  public void disableHeadingHold() {  
    headingHoldEnabled = false;
  }

  /** Llamar cuando se resetea el gyro/navX */
  public void resetHeadingHoldAfterGyroReset() {
    headingSetpointDeg = getRotation2d().getDegrees(); // normalmente 0 tras reset
    headingPID.reset();
    headingHoldEnabled = false; // o true, según lo que quieras
  }

    
    public void setPose(Pose2d pose) {
      simulatedChassis.setSimulationWorldPose(pose);
      simulatedChassis.resetOdometry(pose);
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
      return simulatedChassis.getMeasuredSpeedsRobotRelative(true);
    }

    public void driveRobotRelative(ChassisSpeeds chassis) {
      simulatedChassis.runChassisSpeeds(chassis, new Translation2d(), false, true);
    }

  private boolean wasRotating = false;  // true si en el ciclo anterior |zSpeed| >= deadband

  public void driveWithHeadingHold(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
      double omega;
  
      boolean rotatingNow = Math.abs(zSpeed) >= 0.05;
  
      if (!rotatingNow) { // deadband Z → mantener heading
          // flanco de bajada: veníamos girando y ahora NO
          if (wasRotating || !headingHoldEnabled) {
              // Captura el heading EXACTO en el primer ciclo de reposo
              headingSetpointDeg = getRotation2d().getDegrees();
              headingPID.reset();
              headingHoldEnabled = true;
          }
  
          double currentHeadingDeg = getRotation2d().getDegrees();
          double pidOutput = headingPID.calculate(currentHeadingDeg, headingSetpointDeg);
          pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));
          omega = pidOutput;
      } else {
          // joystick Z manda
          disableHeadingHold();
          omega = zSpeed;
      }
  
      wasRotating = rotatingNow;
  
      drive(xSpeed, ySpeed, omega, fieldOriented);
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }

    /* * * ADDED METHODS * * */
public double deadzone(double num){
  return Math.abs(num) > 0.1 ? num : 0;
}

@SuppressWarnings("unused")
private static double modifyAxis(double num) {
// Square the axis
num = Math.copySign(num * num, num);

return num;
}
}
