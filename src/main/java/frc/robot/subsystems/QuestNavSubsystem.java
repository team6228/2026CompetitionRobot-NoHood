package frc.robot.subsystems;

import java.util.OptionalInt;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.Swerve.DriveTrain;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

    // ---------------- QUEST ----------------
    private final QuestNav questNav = new QuestNav();

    // ---------------- SWERVE ----------------
    private final DriveTrain swerveDrive;

    // ---------------- POSE ----------------
    private Pose2d lastCorrectedPose = new Pose2d();
    private boolean hasPose = false;

    // ---------------- BATTERY ----------------
    private int questBatteryPercent = -1;

    // ---------------- TRACKING ----------------
    private boolean questTracking = false;

    // ---------------- TRANSFORM ----------------
    // Robot merkezinden Quest'e geometrik dönüşüm
    private static final Transform3d ROBOT_TO_QUEST =
        new Transform3d(
            new Translation3d(0.195, 0.285, 0.40),
            new Rotation3d(0.0, 0.0, Math.PI / 2)
        );

    // ---------------- STD DEVS ----------------
    private static final Matrix<N3, N1> QUEST_STD_DEVS =
        VecBuilder.fill(0.07, 0.07, 0.05);

    // ---------------- CONSTRUCTOR ----------------
    public QuestNavSubsystem(DriveTrain driveTrain) {
        this.swerveDrive = driveTrain;
    }

    // ---------------- PERIODIC ----------------
    @Override
    public void periodic() {

        // -------- ZORUNLU: commandPeriodic çağrısı --------
        // v2025-1.0.0 bunu her döngüde ister, olmadan sistem çalışmaz
        questNav.commandPeriodic();

        // -------- BATTERY --------
        OptionalInt batteryOpt = questNav.getBatteryPercent();
        questBatteryPercent = batteryOpt.isPresent() ? batteryOpt.getAsInt() : -1;

        // -------- TRACKING FLAG RESET --------
        questTracking = false;

        // -------- POSE FRAMES --------
        // Tüm okunmamış frame'leri al ve HEPSİNİ pose estimator'a ekle
        PoseFrame[] frames = questNav.getAllUnreadPoseFrames();

        for (PoseFrame frame : frames) {
            if (!frame.isTracking()) continue; // Tracking yoksa bu frame'i atla

            questTracking = true;

            // Quest pose'unu robot merkezine dönüştür (ROBOT_TO_QUEST.inverse())
            Pose3d robotPose3d = frame.questPose3d()
                .transformBy(ROBOT_TO_QUEST.inverse());

            Pose2d robotPose2d = robotPose3d.toPose2d();
            lastCorrectedPose  = robotPose2d;
            hasPose            = true;

            // Doğrudan pose estimator'a gönder — offset yok, setPose reset eder
            swerveDrive.addVisionMeasurement(
                robotPose2d,
                frame.dataTimestamp(),
                QUEST_STD_DEVS
            );
        }

        publishToDashboard();
    }

    // ---------------- DASHBOARD ----------------
    private void publishToDashboard() {
        SmartDashboard.putBoolean("Quest/Tracking",   questTracking);
        SmartDashboard.putBoolean("Quest/Connected",  questNav.isConnected());
        SmartDashboard.putNumber ("Quest/Battery",    questBatteryPercent);

        if (hasPose) {
            SmartDashboard.putNumber("Quest/X",      lastCorrectedPose.getX());
            SmartDashboard.putNumber("Quest/Y",      lastCorrectedPose.getY());
            SmartDashboard.putNumber("Quest/RotDeg", lastCorrectedPose.getRotation().getDegrees());
        }
    }

    // ---------------- PUBLIC API ----------------

    /** Son bilinen düzeltilmiş robot pozisyonu */
    public Pose2d getPose2d() {
        return hasPose ? lastCorrectedPose : new Pose2d();
    }

    /**
     * QuestNav'ı belirli bir field pozisyonuna sıfırlar.
     *
     * Doğru yaklaşım: robot pose'unu Quest frame'ine dönüştürüp
     * questNav.setPose() ile doğrudan QuestNav'a göndermek.
     * Böylece offset birikimine gerek kalmaz.
     */
    public void zeroPose(Pose2d targetRobotPose) {
        // Robot pose'unu → Quest pose'una dönüştür (inverse() YOK!)
        Pose3d targetQuestPose = new Pose3d(targetRobotPose)
            .transformBy(ROBOT_TO_QUEST);

        // QuestNav'a yeni sıfır noktasını bildir
        questNav.setPose(targetQuestPose);

        System.out.println("[QuestNav] zeroPose → " + targetRobotPose);

        
    }

    public boolean isTracking()      { return questTracking; }
    public int     getBatteryPercent() { return questBatteryPercent; }
    public boolean isConnected()     { return questNav.isConnected(); }
}




