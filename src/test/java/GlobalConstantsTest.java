import frc.robot.GlobalConstants;
import org.junit.jupiter.api.Test;

public class GlobalConstantsTest {
  @Test
  public void main() {
    assert (GlobalConstants.FieldMap.Coordinates.REEF_1.getPose().getRotation().getDegrees()
        == 180);
  }
}
