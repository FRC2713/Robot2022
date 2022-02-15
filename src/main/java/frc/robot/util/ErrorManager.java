package frc.robot.util;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.ArrayList;

public class ErrorManager implements Sendable {
  public class Error {
    public String key, value;

    public Error(String key, String value) {
      this.key = key;
      this.value = value;
    }
  }

  private ArrayList<Error> errors = new ArrayList<>();
  private static ErrorManager instance;

  private ErrorManager() {}

  public static ErrorManager getInstance() {
    if (instance == null) {
      instance = new ErrorManager();
    }

    return instance;
  }

  public void addError(Error error) {
    errors.add(error);
  }

  public void addError(String key, String value) {
    errors.add(new Error(key, value));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    errors.forEach(
        (e) -> {
          builder.addStringProperty(e.key, () -> e.value, (unused) -> {});
        });
  }
}
