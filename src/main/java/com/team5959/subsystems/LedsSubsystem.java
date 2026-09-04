// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team5959.subsystems;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * LedsSubsystem — Subsistema de control de LEDs NeoPixel vía Tejuino Board #1
 *
 * Modo DESHABILITADO : Rainbow (arcoíris), re-enviado cada segundo
 * 
 * Modo AUTONOMO : Envía colores del equipo.
 *
 * Modo TELEOPERADO   : Este sistema no sera base al HUB, sino orientado al robot
 *    IDLE              - Color Alianza
 *    Aiming HUB        - YELLOW (BLINK)
 *    Shooting ON       - YELLOW (SOLID)
 *    Intaking ROLLER   - PURPLE (BLINK)
 *    Outaking ROLLER   - PINK (SOLID)
 *    Unjam shooter     - WHITE (BLINK)
 *    Climbing Prep     - GREEN (SOLID)
 *    Climbing Action   - CYAN (BLINK)
 *    Reseting Climbers - MAGENTA (SOLID)
 */
public class LedsSubsystem extends SubsystemBase {

  // ──────────────────────────────────────────────────────────────────────────
  // Tejuino Board — dispositivo CAN número 1
  // ──────────────────────────────────────────────────────────────────────────
  private final TejuinoBoard tejuino = new TejuinoBoard();

  private static final int[] LED_STRIPS = {
      TejuinoBoard.TEJUINO_ONBOARD_LEDS,
      TejuinoBoard.TEJUINO_EXTERNAL_LEDS
  };

  // ──────────────────────────────────────────────────────────────────────────
  // Colores
  // ──────────────────────────────────────────────────────────────────────────
  private static final int RED_R    = 255, RED_G    =   0, RED_B    =   4;
  private static final int BLUE_R   =   0, BLUE_G   =   0, BLUE_B   = 255;
  private static final int TR_R = 125, TR_G = 0, TR_B = 50;

  private static final int YELLOW_R  = 255, YELLOW_G  = 255, YELLOW_B  = 0;
  private static final int PURPLE_R  = 153, PURPLE_G  =  51, PURPLE_B  = 255;
  private static final int PINK_R    = 255, PINK_G    =  20, PINK_B    = 147;
  private static final int WHITE_R   = 255, WHITE_G   = 255, WHITE_B   = 255;
  private static final int GREEN_R   =   0, GREEN_G   = 255, GREEN_B   = 0;
  private static final int CYAN_R    =   0, CYAN_G    = 255, CYAN_B    = 255;
  private static final int MAGENTA_R = 255, MAGENTA_G =   0, MAGENTA_B = 255;


  // ──────────────────────────────────────────────────────────────────────────
  // Parpadeo
  // periodic() se llama cada 20 ms → tickCount sube 1 por ciclo
  // ──────────────────────────────────────────────────────────────────────────
  private int     tickCount = 0;
  private boolean blinkOn   = true;


  // Período de parpadeo en ticks (1 tick = 20 ms)
  //   Máximo: 500 ms = 25 ticks   →   parpadeo lento al inicio
  //   Mínimo:  80 ms =  4 ticks   →   parpadeo rápido al final
  private static final int BLINK_PERIOD_MAX = 25;
  private static final int BLINK_PERIOD_MIN =  4;

  // ──────────────────────────────────────────────────────────────────────────
  // Constructor
  // ──────────────────────────────────────────────────────────────────────────
  public LedsSubsystem() {
    tejuino.init(tejuino.TEJUINO_DEVICE_NUMBER_1);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // periodic — se llama cada 20 ms por el CommandScheduler
  // ──────────────────────────────────────────────────────────────────────────
  @Override
  public void periodic() {
    tickCount++;
      if (DriverStation.isDisabled()) {
      if (tickCount % 50 == 0) setRainbow();
      return;
    }

    if (DriverStation.isAutonomousEnabled()) {
      // Color de alianza sólido, re-enviado cada segundo
      if (tickCount % 50 == 0) set5959Leds();
      return;
    }
    }

  // ──────────────────────────────────────────────────────────────────────────
  // Parpadeo con aceleración progresiva
  //   progress 0.0 = lento (BLINK_PERIOD_MAX)  /  1.0 = rápido (BLINK_PERIOD_MIN)
  // ──────────────────────────────────────────────────────────────────────────
  private void blinkWithColor(double progress, int r, int g, int b) {
    int periodTicks = (int) Math.round(
        BLINK_PERIOD_MAX - progress * (BLINK_PERIOD_MAX - BLINK_PERIOD_MIN)
    );
    periodTicks = Math.max(BLINK_PERIOD_MIN, periodTicks);

    if (tickCount % periodTicks == 0) {
      blinkOn = !blinkOn;
      sendColor(blinkOn ? r : 0, blinkOn ? g : 0, blinkOn ? b : 0);
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Parpadeo fijo (EN MILISEGUNDOS)
  // ──────────────────────────────────────────────────────────────────────────

  private void blinkConstantWithColor(double interval, int r, int g, int b){
    int periodTicks = Math.max(1, (int) Math.round(interval / 20.0));

    if (tickCount % periodTicks == 0) {
      blinkOn = !blinkOn;
      sendColor(blinkOn ? r : 0, blinkOn ? g : 0, blinkOn ? b : 0);
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Helpers de envío CAN
  // ──────────────────────────────────────────────────────────────────────────
  private void sendColor(int r, int g, int b) {
    for (int strip : LED_STRIPS) tejuino.all_led_control(strip, r, g, b);
  }

  private void sendAllianceColor() {
    Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) {
      for (int strip : LED_STRIPS) tejuino.rainbow_effect(strip);
      return;
    }
    if (allianceOpt.get() == DriverStation.Alliance.Red) sendColor(RED_R, RED_G, RED_B);
    else                                                  sendColor(BLUE_R, BLUE_G, BLUE_B);
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Métodos internos para comandos
  // ──────────────────────────────────────────────────────────────────────────
  private void setAllRainbow() {
    for (int strip : LED_STRIPS) tejuino.rainbow_effect(strip);
  }

  private void setLED(int r, int g, int b) {
    sendColor(r, g, b);
  }

  private void setRampedBlink(double progress, int r, int g, int b) {
    blinkWithColor(progress, r, g, b);
  }

  private void setBlink(double interval, int r, int g, int b){
    blinkConstantWithColor(interval, r, g, b);
  }


  // ──────────────────────────────────────────────────────────────────────────
  // Métodos para controlar en RobotContainer
  // ──────────────────────────────────────────────────────────────────────────

  public void setRainbow(){
    setAllRainbow();
  }

  public void set5959Leds(){
    setLED(TR_R, TR_G, TR_B);
  }

  public void setAllianceLeds(){
    sendAllianceColor();
  }

  public void setAimingLeds(){
    setBlink(500, YELLOW_R, YELLOW_G, YELLOW_B);
  }

  public void setShootingLeds(){
    setLED(YELLOW_R, YELLOW_G, YELLOW_B);
  }

  public void setIntakingLeds(){
    setBlink(500, PURPLE_R, PURPLE_G, PURPLE_B);
  }

  public void setOutakingLeds(){
    setLED(PINK_R, PINK_G, PINK_B);
  }

  public void setUnjamLeds(){
    setBlink(700, WHITE_R, WHITE_G, WHITE_B);
  }

  public void setClimbPrepLeds(){
    setLED(GREEN_R, GREEN_G, GREEN_B);
  }

  public void setClimbingLeds(){
    setBlink(350, CYAN_R, CYAN_G, CYAN_B);
  }

  public void setResetClimbersLeds(){
    setLED(MAGENTA_R, MAGENTA_G, MAGENTA_B);
  }

  public void setIdleLeds(){
    if (DriverStation.isFMSAttached()){
      setAllianceLeds();
    }
    else{
      set5959Leds();
    }
  }
}