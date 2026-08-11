package com.team5959.commands;



import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import com.team5959.Constants.SwerveConstants;
import com.team5959.subsystems.SwerveChassis;



    public class AutoAimPassRight extends Command{
    private SwerveChassis swerveChassis; 

    private final Translation2d blueRIGHTPASS = new Translation2d(3.000, 1.500);
    private final Translation2d redRIGHTPASS = new Translation2d(13.500, 6.500);

    private DoubleSupplier xSupplier, ySupplier; 
    private boolean fieldOriented; 

    private final PIDController AimingPID = new PIDController(SwerveConstants.KP_ROT_PATHPLANNER, SwerveConstants.KI_ROT_PATHPLANNER, SwerveConstants.KD_ROT_PATHPLANNER);

      //constructor del chassis
    public AutoAimPassRight(
      SwerveChassis swervecChassis, 
      DoubleSupplier xSupplier, 
      DoubleSupplier ySupplier, 
      boolean fieldOriented
    ) {
   
    this.swerveChassis = swervecChassis; 
    this.xSupplier = xSupplier; 
    this.ySupplier = ySupplier; 
    this.fieldOriented = fieldOriented; 
    addRequirements(swervecChassis);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveChassis.resetHeadingHoldAfterGyroReset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var alliance = DriverStation.getAlliance();
    boolean isBlue = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue;

    Translation2d myRightAlliance = isBlue ? blueRIGHTPASS : redRIGHTPASS;
    Pose2d currentPose = swerveChassis.getPose2d();

    Rotation2d AngleTarget = myRightAlliance.minus(currentPose.getTranslation()).getAngle();
    double RotateToRightPass = AimingPID.calculate(currentPose.getRotation().getDegrees(), AngleTarget.getDegrees());
   
    /* * * ALTERING VALUES * *   */
    //Joystick values -> double 
    double xSpeed = -xSupplier.getAsDouble() * SwerveConstants.MAX_SPEED; 
    double ySpeed = -ySupplier.getAsDouble() * SwerveConstants.MAX_SPEED; 

    //apply deadzone to speed values 
    xSpeed = deadzone(xSpeed); 
    ySpeed = deadzone(ySpeed); 

    //square the speed values to make for smoother acceleration 
    xSpeed = modifyAxis(xSpeed); 
    ySpeed = modifyAxis(ySpeed); 

    swerveChassis.driveWithHeadingHold(xSpeed, ySpeed, RotateToRightPass, fieldOriented);//FIXME SI NO FUNCIONA EL HOLDING REGRESAR A METODO DRIVE

    /*    
    if (fieldOriented) {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, swerveChassis.getRotation2d())
      );
    } else {
      states = SwerveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, zSpeed)
      );
    }

    swerveChassis.setModuleStates(states);
  */
} 
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveChassis.stopModules();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

/* * * ADDED METHODS * * */
public double deadzone(double num){
    return Math.abs(num) > 0.1 ? num : 0;
}

private static double modifyAxis(double num) {
  // Square the axis
  num = Math.copySign(num * num, num);

  return num;
}


}
