// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.subsystems;

import com.team5959.subsystems.SwerveChassis;

import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MapleSwerve extends SubsystemBase{
  /** Creates a new MapleSwerve. */
  private final SelfControlledSwerveDriveSimulation simulatedChassis;
  private final Field2d simField2d;


  public MapleSwerve() {

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

    simField2d = new Field2d();
    SmartDashboard.putData("Simulated Field", simField2d);

  }

  public void drive(Translation2d translation, double rotation, boolean fieldOriented, boolean isOpenLoop){
    this.simulatedChassis.runChassisSpeeds(
      new ChassisSpeeds(translation.getX(), translation.getY(), rotation), 
      new Translation2d(), 
      fieldOriented, 
      true);
  }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        simulatedChassis.runSwerveStates(desiredStates);
    }

    public ChassisSpeeds getMeasuredSpeeds() {
        return simulatedChassis.getMeasuredSpeedsFieldRelative(true);
    }

    public Rotation2d getGyroYaw() {
        return simulatedChassis.getRawGyroAngle();
    }

    public Pose2d getPose() {
        return simulatedChassis.getOdometryEstimatedPose();
    }

    public void setPose(Pose2d pose) {
        simulatedChassis.setSimulationWorldPose(pose);
        simulatedChassis.resetOdometry(pose);
    }

  public void periodic() {
    // This method will be called once per scheduler run
  }
}
