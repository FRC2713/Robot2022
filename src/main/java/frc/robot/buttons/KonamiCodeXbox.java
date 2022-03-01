package frc.robot.buttons;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

public class KonamiCodeXbox extends Button {

  private XboxController xboxController;
  private int buttonNumber;
  private int step = 0;

  public KonamiCodeXbox(XboxController xboxController, int buttonNumber) {
    requireNonNullParam(xboxController, "xbox", "xboxKonamiButton");

    this.xboxController = xboxController;
    buttonNumber = buttonNumber;
  }

  @Override
  public boolean get() {
    // Step 1 - up 1
    if (step == 0) {
      if (xboxController.getPOV() == 0) {
        step = 1;
      }
    } else {
      step = 0;
    }
    // Step 2 - up 2
    if (step == 1) {
      if (xboxController.getPOV() == 0) {
        step = 2;
      }
    } else {
      step = 0;
    }
    // Step 3 - down 1
    if (step == 2) {
      if (xboxController.getPOV() == 180) {
        step = 3;
      }
    } else {
      step = 0;
    }
    // Step 4 - down 2
    if (step == 3) {
      if (xboxController.getPOV() == 180) {
        step = 4;
      }
    } else {
      step = 0;
    }
    // Step 5 - left 1
    if (step == 4) {
      if (xboxController.getPOV() == 270) {
        step = 5;
      }
    } else {
      step = 0;
    }
    // Step 6 - right 1
    if (step == 5) {
      if (xboxController.getPOV() == 90) {
        step = 6;
      }
    } else {
      step = 0;
    }
    // Step 7 - left 2
    if (step == 6) {
      if (xboxController.getPOV() == 270) {
        step = 7;
      }
    } else {
      step = 0;
    }
    // Step 8 - right 2
    if (step == 7) {
      if (xboxController.getPOV() == 90) {
        step = 8;
      }
    } else {
      step = 0;
    }
    // step 9 - a
    if (step == 8) {
      if (xboxController.getAButtonReleased()) {
        step = 9;
      }
    } else {
      step = 0;
    }
    // step 10 - b
    if (step == 9) {
      if (xboxController.getBButtonReleased()) {
        step = 10;
      }
    } else {
      step = 0;
    }
    // step 11 - start
    if (step == 10) {
      if (xboxController.getStartButtonReleased()) {
        step = 11;
      }
    } else {
      step = 0;
    }
    // does this if it worked
    if (step == 11) {
      step = 0;
      return true;
    }
    // does this if it failed
    return false;
  }
}
