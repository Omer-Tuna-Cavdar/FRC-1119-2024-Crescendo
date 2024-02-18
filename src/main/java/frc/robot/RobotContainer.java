package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.ADXL345_I2C.Axes;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.TeleopPivot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.TurnAndX;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
   // private final Joystick atari = new Joystick(1); 
    private final SendableChooser<Command> autoChooser;
  

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kRightStick.value);
    private final JoystickButton fieldCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    private final JoystickButton driveA = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton driveY = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton driveB = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton driveX = new JoystickButton(driver, XboxController.Button.kX.value);
    
    private final JoystickButton opA = new JoystickButton(operator, XboxController.Button.kA.value);
    private final JoystickButton opY = new JoystickButton(operator, XboxController.Button.kY.value);
    private final JoystickButton opB = new JoystickButton(operator, XboxController.Button.kB.value);
    private final JoystickButton opX = new JoystickButton(operator, XboxController.Button.kX.value);



    // private final JoystickButton atariButton1 = new JoystickButton(atari, 1);
    // private final JoystickButton atariButton2 = new JoystickButton(atari, 2);
    // private final JoystickButton atariButton3 = new JoystickButton(atari, 3);

    // private final JoystickButton atariButton4 = new JoystickButton(atari, 4);
    // private final JoystickButton atariButton5 = new JoystickButton(atari, 5);
    // private final JoystickButton atariButton6 = new JoystickButton(atari, 6);

    // private final JoystickButton atariButton7 = new JoystickButton(atari, 7);
    // private final JoystickButton atariButton8 = new JoystickButton(atari, 8);

    

    private final JoystickButton opIntake = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
    private final JoystickButton opOuttake = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);

    // private final JoystickButton atariButton11 = new JoystickButton(atari, 11);
    // private final JoystickButton atariButton12 = new JoystickButton(atari, 12);
    //private final JoystickButton OP = new JoystickButton(operator, XboxController.Button.kRightBumper.value);

    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Collector collector = new Collector();
    private final Pivot pivot = new Pivot();
    private final Launcher launcher = new Launcher();
    private final TurnAndX xLock = new TurnAndX(s_Swerve);
   
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
         s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> fieldCentric.getAsBoolean()
            ));

         pivot.setDefaultCommand(
            new TeleopPivot(
                pivot, 
                () -> -operator.getRawAxis(translationAxis)
            ));

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        
        driveA.whileTrue(s_Swerve.sysIdQuasistatic(Direction.kForward));
        driveX.whileTrue(s_Swerve.sysIdQuasistatic(Direction.kReverse));

        driveB.whileTrue(s_Swerve.sysIdDynamic(Direction.kForward));
        driveY.whileTrue(s_Swerve.sysIdDynamic(Direction.kReverse));

        opA.whileTrue(new InstantCommand(() -> pivot.setPivotPosition(0), pivot)); //Collect+Speaker
        opY.whileTrue(new InstantCommand(() -> pivot.setPivotPosition(0), pivot)); //Amp

        // opB.onTrue(new InstantCommand(() -> launcher.shootSpeaker()));
        // opB.onFalse(new InstantCommand(() -> launcher.launcherStop()));
        // opX.onTrue(new InstantCommand(() -> launcher.shootAmp()));
        // opX.onFalse(new InstantCommand(() -> launcher.launcherStop()));

        opIntake.onTrue(new InstantCommand(() -> collector.collectorIntake()));
        opIntake.onFalse(new InstantCommand(() -> collector.collectorStop()));

        opOuttake.onTrue(new InstantCommand(() -> collector.collectorOuttake()));
        opOuttake.onFalse(new InstantCommand(() -> collector.collectorStop()));

        opA.whileTrue(launcher.sysIdQuasistatic(Direction.kForward));
        opX.whileTrue(launcher.sysIdQuasistatic(Direction.kReverse));

        opB.whileTrue(launcher.sysIdDynamic(Direction.kForward));
        opY.whileTrue(launcher.sysIdDynamic(Direction.kReverse));
       

        //opPivot.onTrue(new InstantCommand(() -> ))

        // atariButton1.onTrue(new InstantCommand(() -> pivot.setPivotPosition(0), pivot)); //SWITCH THIS FOR Collect
        // atariButton2.onTrue(new InstantCommand(() -> pivot.setPivotPosition(0), pivot)); //SWITCH THIS FOR Speaker
        // atariButton3.onTrue(new InstantCommand(() -> pivot.setPivotPosition(0), pivot)); //SWITCH THIS FOR AMP

        // atariButton4.onTrue(new InstantCommand(() -> launcher.shootAmp()));
        // atariButton4.onTrue(new InstantCommand(() -> collector.collectorIntake()));
        // atariButton4.onFalse(new InstantCommand(() -> launcher.launcherStop()));
        // atariButton4.onFalse(new InstantCommand(() -> collector.collectorStop()));

        // atariButton5.onTrue(new InstantCommand(() -> launcher.shootSpeaker()));
        // atariButton5.onTrue(new InstantCommand(() -> collector.collectorOuttake()));
        // atariButton5.onFalse(new InstantCommand(() -> launcher.launcherStop()));
        // atariButton5.onFalse(new InstantCommand(() -> collector.collectorStop()));

        // atariButton7.onTrue(new InstantCommand(() -> collector.collectorIntake()));
        // atariButton7.onFalse(new InstantCommand(() -> collector.collectorStop()));

        // atariButton8.onTrue(new InstantCommand(() -> collector.collectorOuttake()));
        // atariButton8.onFalse(new InstantCommand(() -> collector.collectorStop()));

        // atariButton11.onTrue(new InstantCommand(() -> pivot.pivotUp()));
        // atariButton11.onFalse(new InstantCommand(() -> pivot.pivotStop()));

        // atariButton12.onTrue(new InstantCommand(() -> pivot.pivotDown()));
        // atariButton12.onFalse(new InstantCommand(() -> pivot.pivotStop()));

    }

    public Joystick getDriveController(){
        return driver;
      }

    public Joystick getOperatorController(){
        return operator;
      }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        //return autoChooser.getSelected();
        return new PathPlannerAuto("Score3MidCollect_Mid");
    //    PathPlannerPath path = PathPlannerPath.f romPathFile("StraightLine");

    //    return AutoBuilder.followPath(path);
    }
}
