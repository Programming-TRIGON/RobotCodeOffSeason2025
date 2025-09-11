package frc.trigon.robot.constants;

import edu.wpi.first.wpilibj.util.Color;
import lib.hardware.misc.leds.AddressableLEDStrip;
import lib.hardware.misc.leds.LEDStrip;
import lib.hardware.misc.leds.LEDStripAnimationSettings;

public class LEDConstants {
    public static LEDStripAnimationSettings.ColorFlowSettings COLLECTION_CONFIRMATION_ANIMATION_SETTINGS = new LEDStripAnimationSettings.BlinkSettings(
            Color.kGreen,
            0.5
    );
    public static final LEDStripAnimationSettings.SingleFadeSettings CLIMB_ANIMATION_SETTINGS = new LEDStripAnimationSettings.SingleFadeSettings(Color.kYellow, 0.5);

    private static final boolean INVERTED = false;
    private static final int
            NUMBER_OF_LEDS = 60,
            INDEX_OFFSET = 0,
            LED_STRIP_PORT = 0;
    private static final LEDStrip LED_STRIP = LEDStrip.createAddressableLEDStrip(
            INVERTED,
            NUMBER_OF_LEDS,
            INDEX_OFFSET
    );

    /**
     * Initializes LEDConstants. Needed to be called for the LED strips to be initialized before being used.
     */
    public static void init() {
        AddressableLEDStrip.initiateAddressableLED(LED_STRIP_PORT, NUMBER_OF_LEDS);
    }
}