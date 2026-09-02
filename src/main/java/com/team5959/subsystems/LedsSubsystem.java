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
 * Modo TELEOPERADO   : Imita el estado del Hub del campo (FRC 2026)
 *   - Hub ACTIVO     → color de la alianza (rojo / azul), re-enviado cada segundo
 *   - Hub INACTIVO   → naranja, re-enviado cada segundo
 *   - Últimos 5 s del shift → parpadeo acelerado (500 ms → 80 ms) con el color activo
 *   - End-game (≤ 30 s)     → verde sólido
 *   - Últimos 10 s totales  → parpadeo verde acelerado (500 ms → 80 ms)
 *
 * Lógica del Hub 2026
 * ───────────────────
 *  El dato de juego ("R" o "B") indica qué alianza tuvo el hub INACTIVO primero.
 *  El estado del hub alterna cada 25 s durante el teleop (150 s totales):
 *
 *   Tiempo restante   Turno
 *   > 130 s           Activo (transición / inicio)
 *   130–106 s         Turno 1
 *   105–81 s          Turno 2  (invertido)
 *   80–56 s           Turno 3  (igual turno 1)
 *   55–31 s           Turno 4  (igual turno 2)
 *   ≤ 30 s            End-game → siempre verde
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
  private static final int ORANGE_R = 255, ORANGE_G = 100, ORANGE_B =   0;
  private static final int GREEN_R  =   0, GREEN_G  = 255, GREEN_B  =   0;

  // ──────────────────────────────────────────────────────────────────────────
  // Parpadeo
  // periodic() se llama cada 20 ms → tickCount sube 1 por ciclo
  // ──────────────────────────────────────────────────────────────────────────
  private int     tickCount = 0;
  private boolean blinkOn   = true;

  // Ventana de advertencia antes del fin de cada shift / fin de partido
  private static final double BLINK_WARN_SHIFT_SECS   = 5.0;
  private static final double BLINK_WARN_ENDGAME_SECS = 10.0;

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
      // rainbow_effect es continuo en el Tejuino; lo re-enviamos cada segundo
      // (cada 50 ciclos) para que nunca se congele.
      if (tickCount % 50 == 0) {
        for (int strip : LED_STRIPS) tejuino.rainbow_effect(strip);
      }
      return;
    }

    if (DriverStation.isAutonomousEnabled()) {
      // Color de alianza sólido, re-enviado cada segundo
      if (tickCount % 50 == 0) sendAllianceColor();
      return;
    }

    if (DriverStation.isTeleopEnabled()) {
      handleTeleop();
    }
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Lógica principal del Teleop
  // ──────────────────────────────────────────────────────────────────────────
  private void handleTeleop() {
    double matchTime = DriverStation.getMatchTime();

    // ── END-GAME (≤ 30 s) → Verde ──────────────────────────────────────────
    if (matchTime >= 0.0 && matchTime <= 30.0) {

      if (matchTime <= BLINK_WARN_ENDGAME_SECS) {
        // Últimos 10 s → parpadeo verde cada vez más rápido
        double progress = 1.0 - (matchTime / BLINK_WARN_ENDGAME_SECS); // 0→1
        blinkWithColor(progress, GREEN_R, GREEN_G, GREEN_B);
      } else {
        // Verde sólido — re-enviar cada segundo
        if (tickCount % 50 == 0) sendColor(GREEN_R, GREEN_G, GREEN_B);
      }
      return;
    }

    // ── TELEOP NORMAL → Patrón del Hub ─────────────────────────────────────
    Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) {
      if (tickCount % 50 == 0) {
        for (int strip : LED_STRIPS) tejuino.rainbow_effect(strip);
      }
      return;
    }

    DriverStation.Alliance alliance = allianceOpt.get();
    boolean hubActive = isHubActive(alliance, matchTime);

    // Color base según hub y alianzagr
    int r, g, b;
    if (hubActive) {
      if (alliance == DriverStation.Alliance.Red) { r = RED_R;    g = RED_G;    b = RED_B;    }
      else                                         { r = BLUE_R;   g = BLUE_G;   b = BLUE_B;   }
    } else {
      r = ORANGE_R; g = ORANGE_G; b = ORANGE_B;
    }

    // Segundos que faltan para que cambie el shift actual
    double secsToChange = secondsToNextShift(matchTime);

    if (secsToChange >= 0.0 && secsToChange <= BLINK_WARN_SHIFT_SECS) {
      // Últimos 5 s del shift → parpadeo acelerado
      double progress = 1.0 - (secsToChange / BLINK_WARN_SHIFT_SECS); // 0→1
      blinkWithColor(progress, r, g, b);
    } else {
      // Estado estable — re-enviar cada segundo para que no se congele
      if (tickCount % 50 == 0) sendColor(r, g, b);
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
  // Retorna los segundos que faltan para el próximo cambio de shift.
  // Retorna -1 si estamos en transición (>130 s) o end-game (≤30 s).
  // ──────────────────────────────────────────────────────────────────────────
  private double secondsToNextShift(double matchTime) {
    if (matchTime > 130.0 || matchTime <= 30.0) return -1.0;
    // Bordes inferiores de cada turno (momento en que el shift TERMINA)
    double[] shiftEnds = { 106.0, 81.0, 56.0, 31.0 };
    for (double end : shiftEnds) {
      if (matchTime > end) return matchTime - end;
    }
    return -1.0;
  }

  // ──────────────────────────────────────────────────────────────────────────
  // Lógica del Hub 2026
  // ──────────────────────────────────────────────────────────────────────────
  private boolean isHubActive(DriverStation.Alliance alliance, double matchTime) {
    String gameData = DriverStation.getGameSpecificMessage();

    if (gameData == null || gameData.isEmpty()) return true;

    boolean redInactiveFirst;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default  -> { return true; }
    }

    boolean shift1Active = (alliance == DriverStation.Alliance.Blue)
        ? redInactiveFirst : !redInactiveFirst;

    if (matchTime > 130) return true;
    if (matchTime > 105) return  shift1Active;
    if (matchTime > 80)  return !shift1Active;
    if (matchTime > 55)  return  shift1Active;
    if (matchTime > 30)  return !shift1Active;
    return true; // end-game (no debería llegar aquí)
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
  // Métodos públicos opcionales para comandos externos
  // ──────────────────────────────────────────────────────────────────────────
  public void forceRainbow() {
    for (int strip : LED_STRIPS) tejuino.rainbow_effect(strip);
  }

  public void forceColor(int r, int g, int b) {
    sendColor(r, g, b);
  }

  public void forceBlinkWithColor(double progress, int r, int g, int b) {
    blinkWithColor(progress, r, g, b);
  }
}