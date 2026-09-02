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



    public class AutoAimHub extends Command{
    private SwerveChassis swerveChassis; 

    private final Translation2d blueHUB = new Translation2d(4.620, 4.032);
    private final Translation2d redHUB = new Translation2d(11.925, 4.032);

    private DoubleSupplier xSupplier, ySupplier; 
    private boolean fieldOriented; 

    private final PIDController AimingPID = new PIDController(SwerveConstants.KP_AUTO_HOLDING, SwerveConstants.KI_AUTO_HOLDING, SwerveConstants.KD_AUTO_HOLDING);

      //constructor del chassis
    public AutoAimHub(
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

    Translation2d myHUB = isBlue ? blueHUB : redHUB;
    Pose2d currentPose = swerveChassis.getPose2d();

    Rotation2d AngleTarget = myHUB.minus(currentPose.getTranslation()).getAngle();
    double RotateToHub = AimingPID.calculate(currentPose.getRotation().getDegrees(), AngleTarget.getDegrees());
   
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

    swerveChassis.driveWithHeadingHold(
      (xSpeed/2), 
      (ySpeed/2), 
      RotateToHub, 
      fieldOriented
    );
    //FIXME SI NO FUNCIONA EL HOLDING REGRESAR A METODO DRIVE

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
