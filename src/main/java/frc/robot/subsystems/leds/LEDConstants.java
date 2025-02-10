package frc.robot.subsystems.leds;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;


import static edu.wpi.first.units.Units.Meters;

public final class LEDConstants {
  public static final int LED_PORT = 3;
  private static final Distance LED_SPACING = Meters.of(1 / 60.0); // 60 LEDs per meter
  public static int LED_LENGTH;

  public static Color ALGAE_COLOR = Color.kAquamarine;
  public static Color CORAL_COLOR = Color.kCoral;

  public static Color MORE_COLOR = Color.kGreen;
  public static Color LESS_COLOR = Color.kRed;
  public static Color OK_COLOR = Color.kBlue;

  public static double TRANSLATION_TOLERANCE = 0.1;
  public static double ROTATION_TOLERANCE = 0.1;
  public static double FLASHING_MAX = 1;

  public static int FRONT_LEFT = 0;
  public static int FRONT_RIGHT = 1;
  public static int BACK_LEFT = 2;
  public static int BACK_RIGHT = 3;



  public static final Segment[] SEGMENTS = new Segment[] {new Segment(2, 3, false)};

  public record Segment(int start, int length, boolean reversed) {}

  static {
    for (Segment segment : SEGMENTS) LED_LENGTH += segment.length;
  }
}
