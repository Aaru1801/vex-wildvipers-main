#include "main.h"
#include "autoselector/autoselector.hpp"

// ------------------------------------------------------------
// AUTON NAMES (EDIT THESE)
// ------------------------------------------------------------
static const char* const AUTON_FULL[] = {
  "Right Autonomous",
  "Left Autonomous",
  "Solo AWP Autonomous",
  "Skills",
};

static const char* const AUTON_SHORT[] = {
  "R AUTO",
  "L AUTO",
  "SOLO AWP",
  "SKILLS",
};

// Optional button colors (0xRRGGBB)
static const std::uint32_t AUTON_COLORS[] = {
  0x2B6CB0,
  0x0B5ED7,
  0x3A6B35,
  0xDD6B20,
};

static constexpr std::size_t AUTON_COUNT =
    sizeof(AUTON_FULL) / sizeof(AUTON_FULL[0]);

// ------------------------------------------------------------
// Forward declarations of your real auton functions
// (they already exist in main.cpp)
// ------------------------------------------------------------
void right_autonomous();
void left_autonomous();
void solo_awp_autonomous();
void skills_autonomous();

// ------------------------------------------------------------
// Call this from initialize()
// ------------------------------------------------------------
void start_autoselector() {
  autoselector::Config cfg;
  cfg.count = AUTON_COUNT;
  cfg.names_full = AUTON_FULL;
  cfg.names_short = AUTON_SHORT;
  cfg.colors_hex = AUTON_COLORS;
  cfg.initial_index = 0;
  cfg.show_lock_toggle = true;
  autoselector::start(cfg);
}

// ------------------------------------------------------------
// Call this from autonomous()
// ------------------------------------------------------------
void run_selected_auton() {
  switch (autoselector::selected_index()) {
    case 0: right_autonomous(); break;
    case 1: left_autonomous(); break;
    case 2: solo_awp_autonomous(); break;
    case 3: skills_autonomous(); break;
    default: break;
  }
}