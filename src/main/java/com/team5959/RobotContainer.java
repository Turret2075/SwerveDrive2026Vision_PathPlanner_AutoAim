// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959;

// Import statements for various WPILib classes and custom classes used in the robot code.
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.events.PointTowardsZoneTrigger;

import com.team5959.Constants.ControllerConstants;
import com.team5959.subsystems.ClimberSubsystem;
import com.team5959.subsystems.LedsSubsystem;
import com.team5959.subsystems.PhotonVisionSubsystem;
import com.team5959.subsystems.ShooterSubsystem;
import com.team5959.subsystems.SwerveChassis;
import com.team5959.subsystems.IntakeSubsystem;
import com.team5959.commands.AutoAimHub;
import com.team5959.commands.AutoAimPassLeft;
import com.team5959.commands.AutoAimPassRight;
import com.team5959.commands.AutoShooterStartCmd;
import com.team5959.commands.AutoShooterStopCmd;
import com.team5959.commands.ClimberHoldPosition;
import com.team5959.commands.ClimberHomeCmd;
import com.team5959.commands.ClimberPID;
import com.team5959.commands.ClimberWithJoystick;
import com.team5959.commands.IntakeHoldPosition;
import com.team5959.commands.IntakeInitialPosition;
import com.team5959.commands.IntakeLowPosition;
import com.team5959.commands.IntakeMidPosition;
import com.team5959.commands.IntakeRollerForward;
import com.team5959.commands.IntakeRollerStop;
import com.team5959.commands.ShooterPIDCmd;
import com.team5959.commands.ShooterStopCmd;
import com.team5959.commands.SwerveDriveJoystickCmd;
import com.team5959.commands.SwerveDriveXLockCmd;
import com.team5959.commands.setShooterManualSpeed;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  public Alert noAutoSelected = new Alert("***ADVERTENCIA***            No has selecciono modo autonomo",
      Alert.AlertType.kWarning); // Alerta de advertencia para autónomo no seleccionado

  // Selector de comando autónomo
  private final SendableChooser<Command> autoChooser;
  // Creacion de objetos de SUBSISTEMAS
  private final SwerveChassis swerveChassis = new SwerveChassis();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final PhotonVisionSubsystem PhotonVisionSubsystem = new PhotonVisionSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final LedsSubsystem leds = new LedsSubsystem();

  // Creacion de objetos de CONTROLES
  private final CommandPS4Controller Drivercontrol = new CommandPS4Controller(ControllerConstants.kDriverControllerPort);
  private final CommandPS4Controller Operatorcontrol = new CommandPS4Controller(ControllerConstants.kOperatorControllerPort);

  
  

  public RobotContainer() {

    
    // Registro de comandos nombrados para pathplanner
    NamedCommands.registerCommand("ShooterStart7Cmd", new AutoShooterStartCmd(shooter, 7));
    NamedCommands.registerCommand("ShooterStart4Cmd", new AutoShooterStartCmd(shooter, 4));
    NamedCommands.registerCommand("ShooterStopCmd", new AutoShooterStopCmd(shooter));
    NamedCommands.registerCommand("IntakeInitialPosition", new IntakeInitialPosition(intake));
    NamedCommands.registerCommand("IntakeMidPosition", new IntakeMidPosition(intake));
    NamedCommands.registerCommand("IntakeLowPosition", new IntakeLowPosition(intake));
    NamedCommands.registerCommand("RollerStart", new IntakeRollerForward(intake));
    NamedCommands.registerCommand("RollerStop", new IntakeRollerStop(intake));

   
      

    // Registro de triggers de pathplanner
    new EventTrigger("Prepareforscore").onTrue(Commands.runOnce(() -> {
      System.out.println("Preparando Intake event");
    }));
    // Registro de zonas de enfoque
    new PointTowardsZoneTrigger("Reef").whileTrue(Commands.print("Apuntando a Reef"));

    // Crear un comando PathPlannerAuto usando un archivo de ruta guardado llamado
    // "compAuto3"
    PathPlannerAuto compAuto3Command = new PathPlannerAuto("compAuto3");
    // PathPlannerAuto can also be created with a custom command
    // autoCommand = new PathPlannerAuto(new CustomAutoCommand());

    // Bind to different auto triggers
    compAuto3Command.isRunning().onTrue(Commands.print("Autonomo 3 corriendo"));
    compAuto3Command.timeElapsed(2).onTrue(Commands.print("Han pasado 2 segundos"));
    compAuto3Command.timeRange(2, 4).whileTrue(Commands.print("entre 2 y 4 segundos"));
    compAuto3Command.event("Prepareforscore").onTrue(Commands.print("Pasando por el evento"));
    compAuto3Command.pointTowardsZone("Speaker").onTrue(Commands.print("Viendo al Speaker"));
    compAuto3Command.activePath("Azul4").onTrue(Commands.print("Iniciando path 4"));
    compAuto3Command.nearFieldPosition(new Translation2d(2, 2), 0.5).whileTrue(Commands.print("within 0.5m of (2, 2)"));
    compAuto3Command.inFieldArea(new Translation2d(2, 2), new Translation2d(4, 4))
        .whileTrue(Commands.print("in area of (2, 2) - (4, 4)"));

    // Do all other initialization

    // Build an auto chooser. This will use Commands.none() as the default option.
    // autoChooser = AutoBuilder.buildAutoChooser();

    // For convenience a programmer could change this when going to competition.
    boolean isCompetition = true;

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
        (stream) -> isCompetition
            ? stream.filter(auto -> auto.getName().startsWith("comp"))
            : stream);

    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

    // Configurar los comandos predeterminados de los subsistemas. En este caso, el
    // chasis swerve
    swerveChassis.setDefaultCommand(new SwerveDriveJoystickCmd(swerveChassis,
        () -> Drivercontrol.getLeftY(),
        () -> Drivercontrol.getLeftX(),
        () -> Drivercontrol.getRightX(),
        true));

        climber.setDefaultCommand(new ClimberHoldPosition(climber));

        //intake.setDefaultCommand(new IntakeHoldPosition(intake));


    // Configure the trigger bindings method.
    configureBindings();
  }

  public SwerveChassis getSwerveChassis() {
    return swerveChassis;
  }

  // Configurar los enlaces de botones para los comandos usando lambdas o
  // referencias de método
  private void configureBindings() {

    Drivercontrol.options().onTrue(new InstantCommand(() -> {
      swerveChassis.resetNavx();
      swerveChassis.resetHeadingHoldAfterGyroReset();
    }));

    Drivercontrol.share().onTrue(new InstantCommand(() -> {
      // 1. Resetear navX primero
      // swerveChassis.resetNavx();
      swerveChassis.resetHeadingHoldAfterGyroReset();

      // 2. Ahora que el gyro está a 0, usar esa rotación para odometría
      swerveChassis.resetOdometry(new Pose2d(0, 0, swerveChassis.getRotation2d()));

      // 3. Resetear encoders de los módulos
      swerveChassis.resetDriveEncoders();
    }, swerveChassis));

    //lockPositionButton.whileTrue(new SwerveDriveXLockCmd(swerveChassis));

    //Control y Botones de Driver 1 

    Drivercontrol.cross().onTrue(new IntakeLowPosition(intake)); // mientras presionado
    Drivercontrol.triangle().onTrue(new IntakeInitialPosition(intake)); // al soltar
    Drivercontrol.circle().onTrue(new IntakeMidPosition(intake)); // mientras presionado
    Drivercontrol.L1().whileTrue(new AutoAimHub(swerveChassis, () -> Drivercontrol.getLeftY(),() -> Drivercontrol.getLeftX(),true));
    Drivercontrol.L2().whileTrue(new AutoAimPassLeft(swerveChassis, () -> Drivercontrol.getLeftY(),() -> Drivercontrol.getLeftX(),true));
    Drivercontrol.R2().whileTrue(new AutoAimPassRight(swerveChassis, () -> Drivercontrol.getLeftY(),() -> Drivercontrol.getLeftX(),true));
    Drivercontrol.square().toggleOnTrue(new StartEndCommand(() -> intake.setRollerPIDSpeed(5200), () -> intake.stopRollerMotor(), intake)); 
    Drivercontrol.R1().toggleOnTrue(new StartEndCommand(() -> intake.setRollerPIDSpeed(-5200), () -> intake.stopRollerMotor(), intake)); 
    
    //Control y Botones de Operador 

    Operatorcontrol.L1().onTrue(new ClimberPID(climber, 70)); // mientras presionado
    Operatorcontrol.R1().onTrue(new ClimberPID(climber, 215)); //
    Operatorcontrol.PS().whileTrue(new ClimberHomeCmd(climber)); // mientras presionado

    Operatorcontrol.R2().onTrue(new ClimberWithJoystick(climber, 0.8));
    Operatorcontrol.R2().onFalse(new ClimberHoldPosition(climber));
    Operatorcontrol.L2().onTrue(new ClimberWithJoystick(climber, -0.8));
    Operatorcontrol.L2().onFalse(new ClimberHoldPosition(climber));

    Operatorcontrol.square().toggleOnTrue(
      new StartEndCommand(
        () -> {
          shooter.setShooterPIDSpeed(-4000);
          shooter.setShooterFeederSpeed(-0.8);
          shooter.setShooterIndexerSpeed(-0.85);
        },
        () -> {
          shooter.stopShooter();
          shooter.stopFeeder();
          shooter.stopIndexer();
        },
        shooter, intake
      )
    );
    
    Operatorcontrol.cross().toggleOnTrue(
      new StartEndCommand(
        () -> {
          shooter.setShooterPIDSpeed(4000);
          shooter.setShooterFeederSpeed(0.8);
          shooter.setShooterIndexerSpeed(0.85);
        },
        () -> {
          shooter.stopShooter();
          shooter.stopFeeder();
          shooter.stopIndexer();
        },
        shooter, intake
      )
    ); 


    
    
  }

  public void periodic() {

  }

  public Command getAutonomousCommand() {

    return autoChooser.getSelected();
    // return new AutoFollowTrajectoryCmd(swerveChassis);

  }

  public LedsSubsystem getLeds() {
    return leds;
  }
}