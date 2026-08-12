package com.team5959.commands.SimCommands;



import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import com.team5959.Constants.SwerveConstants;
import com.team5959.subsystems.MapleSwerve;

    public class MapleSwerveDriveJoystickCmd extends Command{
    private MapleSwerve swerveChassis; 

    private DoubleSupplier xSupplier, ySupplier, zSupplier; 
    private boolean fieldOriented; 

      //constructor del chassis
    public MapleSwerveDriveJoystickCmd(MapleSwerve swervecChassis, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier, boolean fieldOriented) {
   
    this.swerveChassis = swervecChassis; 
    this.xSupplier = xSupplier; 
    this.ySupplier = ySupplier; 
    this.zSupplier = zSupplier; 
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
    double rawX = -xSupplier.getAsDouble();
    double rawY = -ySupplier.getAsDouble();
    double rawZ = zSupplier.getAsDouble();

    double cleanX = deadzone(rawX);
    double cleanY = deadzone(rawY);
    double cleanZ = deadzone(rawZ);

    cleanX = modifyAxis(cleanX);
    cleanY = modifyAxis(cleanY);
    cleanZ = modifyAxis(cleanZ);

    double xSpeed = cleanX * SwerveConstants.MAX_SPEED;
    double ySpeed = cleanY * SwerveConstants.MAX_SPEED;
    double zSpeed = -cleanZ * SwerveConstants.MAX_ROTATION;

    swerveChassis.driveWithHeadingHold(xSpeed, ySpeed, zSpeed, fieldOriented);//FIXME SI NO FUNCIONA EL HOLDING REGRESAR A METODO DRIVE

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
