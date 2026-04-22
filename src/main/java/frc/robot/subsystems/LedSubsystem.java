/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedConstants;
import frc.robot.subsystems.Swerve.DriveTrain;

public class LedSubsystem extends SubsystemBase{
    

    //private final AddressableLED led = new AddressableLED(LedConstants.kLEDPwmPort);
    //private final AddressableLEDBuffer buf = new AddressableLEDBuffer(LedConstants.kLedLenght);

    private final DriveTrain drivetrain;
    private final QuestNavSubsystem questNav; 
    private final ShooterTestSubsystem shooter;

    //private LedMode mode = LedMode.IDLE;
    private int tick = 0;
    private boolean blinkState = false;
 
    //Yanıp sönme hızı (periyodik döngü sayısı, 20ms * 25 = 500ms)
    private static final int BLINK_PERIOD = 25;
 
    private final AddressableLED led;
    private final AddressableLEDBuffer buffer;

    public LedSubsystem(DriveTrain m_driveTrain, QuestNavSubsystem m_questNav,
                        ShooterTestSubsystem m_shooter) {
        this.drivetrain = m_driveTrain;
        this.questNav = m_questNav;
        this.shooter = m_shooter;
        drivetrain.setQuestNav(questNav);

        led = new AddressableLED(LedConstants.kLEDDioPort);
        buffer = new AddressableLEDBuffer(LedConstants.kLedLenght);
        led.setLength(LedConstants.kLedLenght);
        led.setData(buffer);
        led.start();
    }


  @Override
    public void periodic() {
        tick++;
 
        if (!questNav.isConnected()) {
            // QuestNav bağlı değil → Kapalı
            setAllLEDs(0, 0, 0);
 
        } else if (shooter.isReady() && questNav.isConnected()) {
            // QuestNav bağlı + Shoot atılıyor → Sabit Beyaz
            setAllLEDs(255, 255, 255);
 
        } else {
            // QuestNav bağlı (normal) → Yanıp Sönen Beyaz
            if (tick >= BLINK_PERIOD) {
                tick = 0;
                blinkState = !blinkState;
            }
            if (blinkState) {
                setAllLEDs(255, 255, 255);
            } else {
                setAllLEDs(0, 0, 0);
            }
        }
 
        led.setData(buffer);
    }

    private void setAllLEDs(int r, int g, int b) {
        for (int i = 0; i < LedConstants.kLedLenght; i++) {
            buffer.setRGB(i, r, g, b);
        }
    }
}
*/