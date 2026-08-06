package frc.robot;

import frc.robot.Constants.OIConstants;
//import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FeederSubsystem;
//import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeWithMotor;
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

    private final DriveTrain          drivetrain                = new DriveTrain();
    private final QuestNavSubsystem   questNav                  = new QuestNavSubsystem(drivetrain);
    private final FeederSubsystem     feederSubsystem           = new FeederSubsystem();
    //private final ClimbSubsystem    climbSubsystem            = new ClimbSubsystem();
    //private final IntakeSubsystem   intakeSubsystem           = new IntakeSubsystem();
    private final ShooterTestSubsystem shooter                  = new ShooterTestSubsystem(drivetrain);
    private final IntakeWithMotor      intakeWithMotorSubsystem = new IntakeWithMotor();
    //private final LedSubsystem ledSubsystem = new LedSubsystem(drivetrain, questNav, shooter);

    public static final CommandXboxController primary = new CommandXboxController(OIConstants.primaryPort);

    public RobotContainer() {

        drivetrain.setQuestNav(questNav);

        // ── Named Commands (PathPlanner) ──────────────────────────────────────
        NamedCommands.registerCommand("FeedShooter",
            new InstantCommand(() -> feederSubsystem.feedShooter(), feederSubsystem));

        NamedCommands.registerCommand("Shoot",
            new ParallelCommandGroup(
                new RunCommand(() -> { shooter.shoot(); shooter.enablePID(); }, shooter),
                new WaitUntilCommand(() -> shooter.isReady())
                    .andThen(new RunCommand(() -> feederSubsystem.feedShooter(), feederSubsystem))
            ));

        NamedCommands.registerCommand("DriveAtTarget",
            new RunCommand(() -> drivetrain.driveAtTarget(0.5, 0.5), drivetrain));

        NamedCommands.registerCommand("StopShooter",
            new InstantCommand(() -> shooter.stopShooter(), shooter));

        NamedCommands.registerCommand("ResetPose",
            new InstantCommand(() -> drivetrain.resetOdometry(drivetrain.getResetPose()), drivetrain));



        NamedCommands.registerCommand("OpenIntake",
            new InstantCommand(() -> intakeWithMotorSubsystem.downIntake(), intakeWithMotorSubsystem));

        NamedCommands.registerCommand("CloseIntake",
            new InstantCommand(() -> intakeWithMotorSubsystem.upIntake(), intakeWithMotorSubsystem));

        NamedCommands.registerCommand("RunIntake",
            new InstantCommand(() -> intakeWithMotorSubsystem.runIntake(), intakeWithMotorSubsystem));


            
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

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

        // ── Start: NavX + Pose sıfırla ────────────────────────────────────────
        primary.start().onTrue(Commands.runOnce(() -> {
            drivetrain.zeroHeading();
            drivetrain.resetPoseToSelected();
        }));

        // ── X: Tekerlekleri X'e kilitle ───────────────────────────────────────
        primary.x().whileTrue(new RunCommand(() -> drivetrain.setX(), drivetrain));

        // ── Right Bumper: Hedefe kilitlenip sürüş ────────────────────────────
        primary.rightBumper().whileTrue(new RunCommand(
                () -> drivetrain.driveAtTarget(
                    -MathUtil.applyDeadband(primary.getLeftY(), 0.1),
                    -MathUtil.applyDeadband(primary.getLeftX(), 0.1)),
                drivetrain))
            .onFalse(new InstantCommand(() -> {
                shooter.stopShooter();
                shooter.disablePID();
            }));

        // ── DPad Down: İleri geri sallanma ───────────────────────────────────
        primary.povDown().whileTrue(
            new RunCommand(() -> drivetrain.oscillate(), drivetrain))
            .onFalse(new InstantCommand(() -> drivetrain.resetOscillate()));

        // ── DPad Up: Shooter toggle (mevcut moda göre RPM) ───────────────────
        primary.povUp().onTrue(new InstantCommand(() -> shooter.toggleShooter(), shooter));

        // ── Left Bumper: İntake aşağı ─────────────────────────────────────────
        primary.leftBumper().onTrue(new InstantCommand(() -> intakeWithMotorSubsystem.toggleIntake(), intakeWithMotorSubsystem));

        // // ── A: Preset A toggle — 15.70° / 2500 RPM ───────────────────────────
        // //   Bir kere bas → hood 15.70°'ye gider
        // //   Tekrar bas   → hood 0°'ye döner, mod interpolasyona döner
        // primary.a().onTrue(new InstantCommand(() -> shooter.togglePresetA(), shooter));

        // // ── B: Preset B toggle — 45° / 3000 RPM ──────────────────────────────
        // //   Bir kere bas → hood 45°'ye gider
        // //   Tekrar bas   → hood 0°'ye döner, mod interpolasyona döner
        // primary.b().onTrue(new InstantCommand(() -> shooter.togglePresetB(), shooter));

        // ── Right Trigger: Moda göre ateşle ──────────────────────────────────
        //   INTERPOLATION modundaysa → mesafeye göre açı + RPM
        //   PRESET_A modundaysa      → 15.70° / 2500 RPM
        //   PRESET_B modundaysa      → 45° / 3000 RPM
        primary.rightTrigger().whileTrue(
            new ParallelCommandGroup(
                new RunCommand(() -> { shooter.shoot(); shooter.enablePID(); }, shooter),
                new WaitUntilCommand(() -> shooter.isReady())
                    .andThen(new RunCommand(() -> feederSubsystem.feedShooter(), feederSubsystem))
            ))
            .onFalse(new InstantCommand(() -> {
                shooter.stopShooter();
                shooter.disablePID();   // Hood boşta kalırken 0°'ye döner (periodic idle)
                feederSubsystem.stopMotors();
            }));

        // ── Left Trigger: Intake çalıştır / durdur ───────────────────────────
        primary.leftTrigger().whileTrue(new InstantCommand(() -> intakeWithMotorSubsystem.runIntake(),  intakeWithMotorSubsystem));
        primary.leftTrigger().onFalse(  new InstantCommand(() -> intakeWithMotorSubsystem.stopIntake(), intakeWithMotorSubsystem));

        // ── Y: Spesifik sabit atış (test/debug) ──────────────────────────────
        primary.y().whileTrue(new RunCommand(() -> shooter.setShooterSpesific(), shooter));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public DriveTrain getDriveTrain() {
        return drivetrain;
    }

    public Command getTimerAutonomousCommand() {
        return drivetrain.runAuto(feederSubsystem, shooter);
    }
}

