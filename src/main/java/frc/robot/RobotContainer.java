package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
//import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.QuestNavSubsystem;
import frc.robot.subsystems.ShooterTestSubsystem;
import frc.robot.subsystems.Swerve.DriveTrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    private final DriveTrain         drivetrain      = new DriveTrain();
    private final QuestNavSubsystem   questNav        = new QuestNavSubsystem(drivetrain);
    private final FeederSubsystem     feederSubsystem = new FeederSubsystem();
    private final ClimbSubsystem      climbSubsystem  = new ClimbSubsystem();
    private final IntakeSubsystem     intakeSubsystem = new IntakeSubsystem();
    private final ShooterTestSubsystem shooter        = new ShooterTestSubsystem(drivetrain);
    //private final LedSubsystem ledSubsystem = new LedSubsystem(drivetrain, questNav, shooter);

    public static final CommandXboxController primary = new CommandXboxController(OIConstants.primaryPort);

    public static boolean isCompB;

    public RobotContainer() {

        // QuestNav'ı DriveTrain'e bağla — setQuestNav çağrısı constructor'dan sonra
        // yapıldığı için resetOdometry içindeki questNav null kontrolü bunu korur.
        drivetrain.setQuestNav(questNav);

        // ── Named Commands (PathPlanner) ──────────────────────────────────────
        NamedCommands.registerCommand("OpenIntake",
            new InstantCommand(() -> intakeSubsystem.openIntake(), intakeSubsystem));

        NamedCommands.registerCommand("CloseIntake",
            new InstantCommand(() -> intakeSubsystem.closeIntake(), intakeSubsystem));

        NamedCommands.registerCommand("RunIntake", 
            new InstantCommand(() -> intakeSubsystem.runIntake(), intakeSubsystem));

        NamedCommands.registerCommand("OpenClimb",
            new InstantCommand(() -> climbSubsystem.climbForward(), climbSubsystem));

        NamedCommands.registerCommand("CloseClimb",
            new InstantCommand(() -> climbSubsystem.climbReverse(), climbSubsystem));

        NamedCommands.registerCommand("FeedShooter",
            new InstantCommand(() -> feederSubsystem.feedShooter(), feederSubsystem));

        NamedCommands.registerCommand("StopIntake",
            new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem));

        NamedCommands.registerCommand("ToggleIntake", 
            new InstantCommand(() -> intakeSubsystem.intakeToggle(), intakeSubsystem));

        NamedCommands.registerCommand("Shoot",
            new ParallelCommandGroup(
                new RunCommand(() -> { shooter.shoot(); shooter.enablePID(); }, shooter),
                new WaitUntilCommand(() -> shooter.isReady())
                    .andThen(new RunCommand(() -> feederSubsystem.feedShooter(), feederSubsystem))
            ));

        NamedCommands.registerCommand("DriveAtTarget", 
            new RunCommand(() -> drivetrain.driveAtTarget(
                0.5,0.5),
                drivetrain
            ));

        NamedCommands.registerCommand("StopShooter", 
            new InstantCommand(() -> shooter.stopShooter(), shooter));
        
        NamedCommands.registerCommand("ResetPose", 
            new InstantCommand(() -> drivetrain.resetOdometry(drivetrain.getResetPose()), drivetrain));
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putBoolean("compB", isCompB);

        // ── Default Drive Command ─────────────────────────────────────────────
        drivetrain.setDefaultCommand(
            new RunCommand(
                () -> {
                    double ySpeed = -MathUtil.applyDeadband(primary.getLeftY(),  OIConstants.driveDeadband);
                    double xSpeed = -MathUtil.applyDeadband(primary.getLeftX(),  OIConstants.driveDeadband);
                    double rot    = -MathUtil.applyDeadband(primary.getRightX(), OIConstants.driveDeadband);
                    drivetrain.drive(ySpeed, xSpeed, rot, true);
                },
                drivetrain));
    }

    private void configureBindings() {

        // ── Start butonu: NavX + Pose sıfırlama ──────────────────────────────
        // Tek komutta toplandı — üçlü bağımsız onTrue yerine.
        // Sıra: zeroHeading() → resetPoseToSelected() (içinde questNav.resetToPose() var)
        primary.start().onTrue(Commands.runOnce(() -> {
            drivetrain.zeroHeading();          // NavX sıfırla
            drivetrain.resetPoseToSelected();  // poseEstimator + QuestNav offset'i sıfırla
        }));

        // ── X: Tekerlekleri X'e kilitle (defans) ─────────────────────────────
        primary.x().whileTrue(new RunCommand(() -> drivetrain.setX(), drivetrain));

        // ── Right Bumper: Hedefe kilitlenerek sürüş + Shoot ──────────────────────────
        

        // ── DPad Up: Hedefe Kitlenerek sürüş ──────────────────────────
        primary.rightBumper().whileTrue(new RunCommand(
                () -> drivetrain.driveAtTarget(
                    -MathUtil.applyDeadband(primary.getLeftY(), 0.1),
                    -MathUtil.applyDeadband(primary.getLeftX(), 0.1)),
                drivetrain)
        )
        .onFalse(new InstantCommand(() -> {
            shooter.stopShooter();
            shooter.disablePID();
        }));

        // ── DPad Down: İleri Geri ──────────────────────────
        primary.povDown().whileTrue(
            new RunCommand(() -> drivetrain.oscillate(), drivetrain))
            .onFalse(new InstantCommand(() -> drivetrain.resetOscillate()));

        
        // ── Left Bumper: Intake toggle ────────────────────────────────────────
        primary.leftBumper().onTrue(new InstantCommand(() -> intakeSubsystem.intakeToggle(), intakeSubsystem));

        // ── POV Up: Tırmanma toggle ───────────────────────────────────────────

        // ── POV Down: Intake kapat / aç ──────────────────────────────────────

        // ── Right Trigger: Ateşleme ───────────────────────────────────────────
        primary.rightTrigger().whileTrue(
            new ParallelCommandGroup(
                new RunCommand(() -> { shooter.shoot(); shooter.enablePID(); }, shooter),
                new InstantCommand(() -> climbSubsystem.compressorDisable(), climbSubsystem),
                new WaitUntilCommand(() -> shooter.isReady())
                    .andThen(new RunCommand(() -> feederSubsystem.feedShooter(), feederSubsystem))
            ))
            .onFalse(new InstantCommand(() -> {
                shooter.stopShooter();
                shooter.disablePID();
                feederSubsystem.stopMotors();
                climbSubsystem.compressorEnable();
                
                
            }));

        // ── Left Trigger: Intake çalıştır / durdur ───────────────────────────
        primary.leftTrigger().whileTrue( new InstantCommand(() -> intakeSubsystem.runIntake(),  intakeSubsystem));
        primary.leftTrigger().onFalse(   new InstantCommand(() -> intakeSubsystem.stopIntake(), intakeSubsystem));

        // ── B: Kompresör toggle ───────────────────────────────────────────────
        primary.b().onTrue(new InstantCommand(() -> climbSubsystem.compressorToggle(), climbSubsystem));
        /*primary.b().onTrue(new RunCommand(() -> {
                        climbSubsystem.compressorToggle();

                        if(climbSubsystem.isCompReady()){
                                isCompB=true;
                        }
                        else{
                            isCompB=false;
                        }
                    }, 
                    climbSubsystem)
                );
            */
        // ── A: Shooter toggle ─────────────────────────────────────────────────
        primary.a().onTrue(new InstantCommand(() -> shooter.toggleShooter(), shooter));

        primary.y().whileTrue(new RunCommand(() -> {
                shooter.setShooterSpesific();
            }, 
            shooter
        ));
    }

    public Command getAutonomousCommand() {
        
        return autoChooser.getSelected();
    }

    public DriveTrain getDriveTrain(){
        return drivetrain;
    }

    public IntakeSubsystem getIntake(){
        return intakeSubsystem;
    }

    public Command getTimerAutonomousCommand() {
        return drivetrain.runAuto(feederSubsystem, shooter);
    }
}