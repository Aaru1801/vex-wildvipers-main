#pragma once

// This library is C++-only.
// If you see a huge wall of compiler errors from this header, you likely included it from a .c file
// or within an `extern "C"` block. Include it from a .cpp file instead.
#ifndef __cplusplus
#error "autoselector.hpp is a C++ header. Include it from a .cpp file (not .c, and not inside extern \"C\")."
#endif

#include <cstddef>
#include <cstdint>

// Some teams (accidentally) include C++ headers inside an `extern "C"` block.
// Wrapping in `extern "C++"` makes this header resilient and avoids a huge cascade of errors.
extern "C++" {

namespace autoselector {

struct Config {
    // Required: number of autonomous routines
    std::size_t count = 0;

    // Required: full display names (size == count)
    const char* const* names_full = nullptr;

    // Optional: short names for buttons (size == count). If null, uses names_full.
    const char* const* names_short = nullptr;

    // Optional: per-auton button colors as 0xRRGGBB (size == count). If null, uses defaults.
    const std::uint32_t* colors_hex = nullptr;

    // Optional: which auton is preselected on boot
    std::size_t initial_index = 0;

    // Optional: if false, hides the LOCK/UNLOCK button and keeps selector unlocked
    bool show_lock_toggle = true;
};

// Build UI (if needed) and show selector screen.
// Safe to call multiple times.
void start(const Config& cfg);

// Current selection.
std::size_t selected_index();

// Lock state (prevents selection changes from touch when locked).
bool locked();
void set_locked(bool is_locked);

// Programmatic selection. If locked, this is a no-op and returns false.
bool set_selected_index(std::size_t idx);

// Update a button's color at runtime (0xRRGGBB). Returns false if idx out of range.
bool set_color_hex(std::size_t idx, std::uint32_t hex);

} // namespace autoselector

} // extern "C++"
