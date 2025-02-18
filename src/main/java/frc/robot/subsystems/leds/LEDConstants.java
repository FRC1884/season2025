package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.util.Color;

public final class LEDConstants {
  public static final int LED_PORT = 0;
  private static final Distance LED_SPACING = Meters.of(1 / 60.0); // 60 LEDs per meter
  public static int LED_LENGTH;

  public static Color ALGAE_COLOR = Color.kAquamarine;
  public static Color CORAL_COLOR = Color.kCoral;

  public static Color MORE_COLOR = Color.kGreen;
  public static Color LESS_COLOR = Color.kRed;
  public static Color OK_COLOR = Color.kBlue;

  public static Time BREATHE_SPEED = Second.of(2.5);

  public static double TRANSLATION_TOLERANCE = 0.1;
  public static double ROTATION_TOLERANCE = 0.1;
  public static double FLASHING_MAX = 1;

  public static int FRONT_LEFT = 0;
  public static int FRONT_RIGHT = 1;
  public static int BACK_LEFT = 2;
  public static int BACK_RIGHT = 3;

  public static final Segment[] SEGMENTS =
      new Segment[] {
        new Segment(0, 30, false),
        new Segment(30, 34, false),
        new Segment(64, 30, false),
        new Segment(94, 30, false),
        new Segment(124, 34, false),
        new Segment(158, 30, false)
      };

  public record Segment(int start, int length, boolean reversed) {}

  static {
    for (Segment segment : SEGMENTS) LED_LENGTH += segment.length;
  }
}
