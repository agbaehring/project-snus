// =========================
// Zumo Roomba Configuration
// =========================

// ---- Movement Parameters ----
#define BASE_SPEED        150
#define TURN_SPEED        120
#define TURN_ANGLE        90.0
#define TURN_SMOOTHING    true
#define TURN_TIMEOUT      6000

// ---- Sensor Thresholds ----
#define LINE_UPPER_THRESHOLD    600
#define LINE_LOWER_THRESHOLD    400
#define WALL_THRESHOLD    5
#define SIDE_THRESHOLD    3

// ---- Timing and Control ----
#define TURN_DELAY_PER_DEG 6
#define STOP_DELAY         300
#define WHEEL_CIRC         38

// ---- OLED Display ----
#define SHOW_STATUS true

// ---- Behavior Toggles ----
#define ENABLE_LINES true
#define ENABLE_WALLS true