"""ESPHome external component for Trinamic TMC2208 / TMC2209 stepper drivers (ESP32 only).

Step generation: GPTimer ISR with integer ramp planner.
Driver access: optional single-wire UART (register configuration + diagnostics).
"""

import math

from esphome import automation, pins
import esphome.codegen as cg
from esphome.components import uart
from esphome.components.esp32 import (
    add_idf_sdkconfig_option,
    include_builtin_idf_component,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_ACCELERATION,
    CONF_ADDRESS,
    CONF_DECELERATION,
    CONF_DIR_PIN,
    CONF_DIRECTION,
    CONF_ENABLE_PIN,
    CONF_ID,
    CONF_MAX_SPEED,
    CONF_MODE,
    CONF_POSITION,
    CONF_RX_PIN,
    CONF_SOURCE,
    CONF_SPEED,
    CONF_STEP_PIN,
    CONF_THRESHOLD,
    CONF_TIMEOUT,
    CONF_TX_PIN,
)
import esphome.final_validate as fv

CODEOWNERS = []
DEPENDENCIES = ["esp32"]
MULTI_CONF = True

CONF_TMC22XX_ID = "tmc22xx_id"
CONF_UART_ID = "uart_id"

CONF_DRIVER = "driver"
CONF_MS1_PIN = "ms1_pin"
CONF_MS2_PIN = "ms2_pin"
CONF_DIAG_PIN = "diag_pin"
CONF_MICROSTEPS = "microsteps"
CONF_INTERPOLATION = "interpolation"
CONF_RUN_CURRENT = "run_current"
CONF_HOLD_CURRENT = "hold_current"
CONF_RSENSE = "rsense"
CONF_STANDSTILL_DELAY = "standstill_delay"
CONF_CHOPPER = "chopper"
CONF_STEALTHCHOP_THRESHOLD = "stealthchop_threshold"
CONF_INVERT_DIRECTION = "invert_direction"
CONF_STEPS_PER_UNIT = "steps_per_unit"
CONF_MIN_POSITION = "min_position"
CONF_MAX_POSITION = "max_position"
CONF_IDLE_DISABLE_DELAY = "idle_disable_delay"
CONF_ENABLE_ON_BOOT = "enable_on_boot"
CONF_POLL_INTERVAL = "poll_interval"
CONF_REPORT_INTERVAL = "report_interval"
CONF_STALLGUARD_THRESHOLD = "stallguard_threshold"
CONF_STOP_ON_STALL = "stop_on_stall"
CONF_HOMING = "homing"
CONF_ENDSTOP_PIN = "endstop_pin"
CONF_SLOW_SPEED = "slow_speed"
CONF_BACKOFF = "backoff"
CONF_MAX_TRAVEL = "max_travel"
CONF_DEBOUNCE = "debounce"
CONF_HOME_POSITION = "home_position"
CONF_PARK_POSITION = "park_position"
CONF_ON_BOOT = "on_boot"
CONF_DISTANCE = "distance"
CONF_ON_MOVE_COMPLETE = "on_move_complete"
CONF_ON_HOMING_COMPLETE = "on_homing_complete"
CONF_ON_HOMING_FAILED = "on_homing_failed"
CONF_ON_STALL = "on_stall"
CONF_ON_DRIVER_ERROR = "on_driver_error"

MODE_UART = "uart"
MODE_PINS = "pins"
DRIVER_TMC2208 = "tmc2208"
DRIVER_TMC2209 = "tmc2209"
SOURCE_ENDSTOP = "endstop"
SOURCE_STALLGUARD = "stallguard"
CHOPPER_STEALTHCHOP = "stealthchop"
CHOPPER_SPREADCYCLE = "spreadcycle"

# Must match STEPGEN_MAX_HZ in tmc22xx_stepgen.h
MAX_STEP_RATE = 100000

tmc22xx_ns = cg.esphome_ns.namespace("tmc22xx")
TMC22XX = tmc22xx_ns.class_("TMC22XX", cg.Component)
DriverType = tmc22xx_ns.enum("DriverType")
HomingSource = tmc22xx_ns.enum("HomingSource")

DRIVER_TYPES = {
    DRIVER_TMC2208: DriverType.DRIVER_TMC2208,
    DRIVER_TMC2209: DriverType.DRIVER_TMC2209,
}
HOMING_SOURCES = {
    SOURCE_ENDSTOP: HomingSource.HOMING_SOURCE_ENDSTOP,
    SOURCE_STALLGUARD: HomingSource.HOMING_SOURCE_STALLGUARD,
}

# Microstep settings selectable through the MS1/MS2 pins (stand-alone operation)
PIN_MICROSTEPS = {
    DRIVER_TMC2208: (2, 4, 8, 16),
    DRIVER_TMC2209: (8, 16, 32, 64),
}

# Options that need register access and therefore the UART
UART_ONLY_KEYS = (
    CONF_UART_ID,
    CONF_ADDRESS,
    CONF_RUN_CURRENT,
    CONF_HOLD_CURRENT,
    CONF_RSENSE,
    CONF_INTERPOLATION,
    CONF_CHOPPER,
    CONF_STEALTHCHOP_THRESHOLD,
    CONF_STANDSTILL_DELAY,
    CONF_POLL_INTERVAL,
    CONF_STALLGUARD_THRESHOLD,
    CONF_DIAG_PIN,
    CONF_STOP_ON_STALL,
    CONF_ON_STALL,
    CONF_ON_DRIVER_ERROR,
)
# Options that only exist on the TMC2209 (StallGuard4)
TMC2209_ONLY_KEYS = (
    CONF_STALLGUARD_THRESHOLD,
    CONF_DIAG_PIN,
    CONF_STOP_ON_STALL,
    CONF_ON_STALL,
)
UART_DEFAULTS = {
    CONF_RSENSE: 0.11,
    CONF_RUN_CURRENT: 0.5,
    CONF_INTERPOLATION: True,
    CONF_CHOPPER: CHOPPER_STEALTHCHOP,
}


def _validate_direction(value):
    """forward/backward (or a boolean, true = forward) -> bool."""
    if isinstance(value, bool):
        return value
    value = cv.one_of("forward", "backward", "positive", "negative", lower=True)(value)
    return value in ("forward", "positive")


def _homing_direction(value):
    return cv.one_of("positive", "negative", lower=True)(value)


def _milliamps(value):
    """Current with unit -> mA as float (lambdas return mA as well)."""
    return cv.All(cv.current, cv.Range(min=0.0, max=2.8))(value) * 1000.0


def _max_rms_current(rsense: float) -> float:
    # CS = 31, V_FS = 325 mV: I_rms = V_FS / (R_sense + 20 mOhm) / sqrt(2)
    return 0.325 / (rsense + 0.02) / math.sqrt(2)


def _validate_homing(config):
    if config[CONF_SOURCE] == SOURCE_ENDSTOP:
        if CONF_ENDSTOP_PIN not in config:
            raise cv.Invalid(
                "homing with 'source: endstop' requires 'endstop_pin'",
                path=[CONF_ENDSTOP_PIN],
            )
    elif CONF_ENDSTOP_PIN in config:
        raise cv.Invalid(
            "'endstop_pin' is not used with 'source: stallguard'",
            path=[CONF_ENDSTOP_PIN],
        )
    elif CONF_SLOW_SPEED in config:
        raise cv.Invalid(
            "'slow_speed' (second approach) is not supported with 'source: stallguard': "
            "StallGuard4 gives no usable signal at low velocity",
            path=[CONF_SLOW_SPEED],
        )
    if CONF_SLOW_SPEED in config:
        if config[CONF_BACKOFF] <= 0:
            raise cv.Invalid(
                "'slow_speed' requires a 'backoff' distance greater than 0",
                path=[CONF_BACKOFF],
            )
        if config[CONF_SLOW_SPEED] > config[CONF_SPEED]:
            raise cv.Invalid(
                "'slow_speed' must not be higher than 'speed'", path=[CONF_SLOW_SPEED]
            )
    return config


HOMING_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_SOURCE, default=SOURCE_ENDSTOP): cv.one_of(
                *HOMING_SOURCES, lower=True
            ),
            cv.Optional(CONF_ENDSTOP_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_DIRECTION, default="negative"): _homing_direction,
            cv.Required(CONF_SPEED): cv.positive_not_null_float,
            cv.Optional(CONF_SLOW_SPEED): cv.positive_not_null_float,
            cv.Optional(CONF_BACKOFF, default=0.0): cv.positive_float,
            cv.Required(CONF_MAX_TRAVEL): cv.positive_not_null_float,
            cv.Optional(
                CONF_TIMEOUT, default="60s"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_DEBOUNCE, default="1ms"): cv.All(
                cv.positive_time_period_microseconds,
                cv.Range(max=cv.TimePeriod(milliseconds=100)),
            ),
            cv.Optional(CONF_HOME_POSITION, default=0.0): cv.float_,
            cv.Optional(CONF_PARK_POSITION): cv.float_,
            cv.Optional(CONF_ON_BOOT, default=False): cv.boolean,
        }
    ),
    _validate_homing,
)


def _validate_config(config):
    driver = config[CONF_DRIVER]
    mode = config[CONF_MODE]

    if driver == DRIVER_TMC2208:
        for key in TMC2209_ONLY_KEYS:
            if key in config:
                raise cv.Invalid(
                    f"'{key}' is not available for the TMC2208: the chip has no StallGuard and no "
                    "stall output on DIAG. Use 'driver: tmc2209' or an end stop for homing.",
                    path=[key],
                )
        if config.get(CONF_ADDRESS, 0) != 0:
            raise cv.Invalid(
                "The TMC2208 has no address pins, its UART address is always 0. "
                "Use one UART bus per TMC2208.",
                path=[CONF_ADDRESS],
            )

    if mode == MODE_PINS:
        for key in UART_ONLY_KEYS:
            if key in config:
                raise cv.Invalid(
                    f"'{key}' requires register access and is only available with 'mode: uart'",
                    path=[key],
                )
        has_ms1 = CONF_MS1_PIN in config
        has_ms2 = CONF_MS2_PIN in config
        if has_ms1 != has_ms2:
            raise cv.Invalid("'ms1_pin' and 'ms2_pin' must be configured together")
        if has_ms1:
            if CONF_MICROSTEPS not in config:
                raise cv.Invalid(
                    "'microsteps' is required when 'ms1_pin'/'ms2_pin' are configured",
                    path=[CONF_MICROSTEPS],
                )
            if config[CONF_MICROSTEPS] not in PIN_MICROSTEPS[driver]:
                allowed = ", ".join(str(x) for x in PIN_MICROSTEPS[driver])
                raise cv.Invalid(
                    f"The {driver.upper()} only supports {allowed} microsteps via MS1/MS2",
                    path=[CONF_MICROSTEPS],
                )
        elif CONF_MICROSTEPS in config:
            raise cv.Invalid(
                "'microsteps' has no effect in 'mode: pins' without 'ms1_pin'/'ms2_pin' "
                "(resolution is set by the module jumpers; adjust 'steps_per_unit' instead)",
                path=[CONF_MICROSTEPS],
            )
    else:
        if CONF_UART_ID not in config:
            raise cv.Invalid(
                "'mode: uart' requires 'uart_id' (use 'mode: pins' for STEP/DIR only operation)",
                path=[CONF_UART_ID],
            )
        for key in (CONF_MS1_PIN, CONF_MS2_PIN):
            if key in config:
                raise cv.Invalid(
                    f"'{key}' is not used in 'mode: uart' (microsteps are set via register; on the "
                    "TMC2209 the MS1/MS2 pins select the UART address and must be hard wired)",
                    path=[key],
                )
        config = config.copy()
        for key, value in UART_DEFAULTS.items():
            config.setdefault(key, value)
        config.setdefault(CONF_MICROSTEPS, 16)
        config.setdefault(CONF_ADDRESS, 0)
        config.setdefault(CONF_STANDSTILL_DELAY, cv.TimePeriod(seconds=2))
        config.setdefault(CONF_POLL_INTERVAL, cv.TimePeriod(seconds=2))
        config.setdefault(CONF_HOLD_CURRENT, config[CONF_RUN_CURRENT] / 2)

        i_max = _max_rms_current(config[CONF_RSENSE])
        if config[CONF_RUN_CURRENT] > i_max:
            raise cv.Invalid(
                f"run_current {config[CONF_RUN_CURRENT]:.2f} A exceeds the full scale current of "
                f"{i_max:.2f} A (RMS) for rsense = {config[CONF_RSENSE] * 1000:.0f} mOhm",
                path=[CONF_RUN_CURRENT],
            )
        if config[CONF_HOLD_CURRENT] > config[CONF_RUN_CURRENT]:
            raise cv.Invalid(
                "hold_current must not be higher than run_current",
                path=[CONF_HOLD_CURRENT],
            )
        if (
            CONF_STEALTHCHOP_THRESHOLD in config
            and config[CONF_CHOPPER] != CHOPPER_STEALTHCHOP
        ):
            raise cv.Invalid(
                "'stealthchop_threshold' only applies to 'chopper: stealthchop'",
                path=[CONF_STEALTHCHOP_THRESHOLD],
            )
        if config.get(CONF_STOP_ON_STALL) or CONF_ON_STALL in config:
            if (
                CONF_DIAG_PIN not in config
                or CONF_STALLGUARD_THRESHOLD not in config
            ):
                raise cv.Invalid(
                    "Stall detection requires 'diag_pin' and 'stallguard_threshold'"
                )
            if config[CONF_CHOPPER] != CHOPPER_STEALTHCHOP:
                raise cv.Invalid(
                    "StallGuard4 only works in StealthChop; use 'chopper: stealthchop'",
                    path=[CONF_CHOPPER],
                )

    steps_per_unit = config[CONF_STEPS_PER_UNIT]
    if config[CONF_MAX_SPEED] * steps_per_unit > MAX_STEP_RATE:
        raise cv.Invalid(
            f"max_speed results in {config[CONF_MAX_SPEED] * steps_per_unit:.0f} steps/s, "
            f"the step generator is limited to {MAX_STEP_RATE} steps/s",
            path=[CONF_MAX_SPEED],
        )
    if (
        CONF_MIN_POSITION in config
        and CONF_MAX_POSITION in config
        and config[CONF_MIN_POSITION] >= config[CONF_MAX_POSITION]
    ):
        raise cv.Invalid(
            "min_position must be lower than max_position", path=[CONF_MIN_POSITION]
        )

    if (homing := config.get(CONF_HOMING)) is not None:
        if homing[CONF_SPEED] * steps_per_unit > MAX_STEP_RATE:
            raise cv.Invalid(
                "homing speed exceeds the step generator limit",
                path=[CONF_HOMING, CONF_SPEED],
            )
        if homing[CONF_SOURCE] == SOURCE_STALLGUARD:
            if driver != DRIVER_TMC2209:
                raise cv.Invalid(
                    "Sensorless homing ('source: stallguard') requires 'driver: tmc2209'. "
                    "The TMC2208 has no StallGuard; use 'source: endstop'.",
                    path=[CONF_HOMING, CONF_SOURCE],
                )
            if mode != MODE_UART:
                raise cv.Invalid(
                    "'source: stallguard' requires 'mode: uart'",
                    path=[CONF_HOMING, CONF_SOURCE],
                )
            if (
                CONF_DIAG_PIN not in config
                or CONF_STALLGUARD_THRESHOLD not in config
            ):
                raise cv.Invalid(
                    "'source: stallguard' requires 'diag_pin' and 'stallguard_threshold'",
                    path=[CONF_HOMING, CONF_SOURCE],
                )
    return config


def _automation_schema():
    return automation.validate_automation({})


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(TMC22XX),
            cv.Optional(CONF_DRIVER, default=DRIVER_TMC2208): cv.one_of(
                *DRIVER_TYPES, lower=True
            ),
            cv.Optional(CONF_MODE, default=MODE_UART): cv.one_of(
                MODE_UART, MODE_PINS, lower=True
            ),
            # pins
            cv.Required(CONF_STEP_PIN): pins.internal_gpio_output_pin_schema,
            cv.Required(CONF_DIR_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_ENABLE_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_MS1_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_MS2_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DIAG_PIN): pins.internal_gpio_input_pin_schema,
            # motion
            cv.Required(CONF_MAX_SPEED): cv.positive_not_null_float,
            cv.Optional(CONF_ACCELERATION, default=1000.0): cv.positive_not_null_float,
            cv.Optional(CONF_DECELERATION): cv.positive_not_null_float,
            cv.Optional(CONF_STEPS_PER_UNIT, default=1.0): cv.positive_not_null_float,
            cv.Optional(CONF_MIN_POSITION): cv.float_,
            cv.Optional(CONF_MAX_POSITION): cv.float_,
            cv.Optional(CONF_INVERT_DIRECTION, default=False): cv.boolean,
            cv.Optional(CONF_ENABLE_ON_BOOT, default=True): cv.boolean,
            cv.Optional(CONF_IDLE_DISABLE_DELAY): cv.positive_time_period_milliseconds,
            cv.Optional(
                CONF_REPORT_INTERVAL, default="500ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_MICROSTEPS): cv.one_of(
                1, 2, 4, 8, 16, 32, 64, 128, 256, int=True
            ),
            # driver registers (mode: uart) - defaults are applied in _validate_config
            cv.Optional(CONF_UART_ID): cv.use_id(uart.UARTComponent),
            cv.Optional(CONF_ADDRESS): cv.int_range(min=0, max=3),
            cv.Optional(CONF_RSENSE): cv.All(
                cv.resistance, cv.Range(min=0.05, max=1.0)
            ),
            cv.Optional(CONF_RUN_CURRENT): cv.All(
                cv.current, cv.Range(min=0.05, max=2.8)
            ),
            cv.Optional(CONF_HOLD_CURRENT): cv.All(
                cv.current, cv.Range(min=0.0, max=2.8)
            ),
            cv.Optional(CONF_INTERPOLATION): cv.boolean,
            cv.Optional(CONF_CHOPPER): cv.one_of(
                CHOPPER_STEALTHCHOP, CHOPPER_SPREADCYCLE, lower=True
            ),
            cv.Optional(CONF_STEALTHCHOP_THRESHOLD): cv.positive_not_null_float,
            cv.Optional(CONF_STANDSTILL_DELAY): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(max=cv.TimePeriod(milliseconds=5570)),
            ),
            cv.Optional(CONF_POLL_INTERVAL): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=100)),
            ),
            # TMC2209 only
            cv.Optional(CONF_STALLGUARD_THRESHOLD): cv.int_range(min=0, max=255),
            cv.Optional(CONF_STOP_ON_STALL): cv.boolean,
            cv.Optional(CONF_HOMING): HOMING_SCHEMA,
            # automations
            cv.Optional(CONF_ON_MOVE_COMPLETE): _automation_schema(),
            cv.Optional(CONF_ON_HOMING_COMPLETE): _automation_schema(),
            cv.Optional(CONF_ON_HOMING_FAILED): _automation_schema(),
            cv.Optional(CONF_ON_STALL): _automation_schema(),
            cv.Optional(CONF_ON_DRIVER_ERROR): _automation_schema(),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on_esp32,
    _validate_config,
)


def _final_validate(config):
    if CONF_UART_ID not in config:
        return config
    full = fv.full_config.get()
    uart_id = config[CONF_UART_ID]
    uart_path = full.get_path_for_id(uart_id)[:-1]
    uart_config = full.get_config_for_path(uart_path)
    for pin in (CONF_TX_PIN, CONF_RX_PIN):
        if pin not in uart_config:
            raise cv.Invalid(
                f"tmc22xx needs both tx_pin and rx_pin on uart '{uart_id}' "
                "(single-wire UART: TX via 1 kOhm to PDN_UART, RX directly to PDN_UART)"
            )
    # Bus sharing: TMC2209 by address, TMC2208 needs a dedicated bus
    users = full.data.setdefault("tmc22xx_uart_users", {}).setdefault(str(uart_id), [])
    for driver, address in users:
        if DRIVER_TMC2208 in (driver, config[CONF_DRIVER]):
            raise cv.Invalid(
                f"uart '{uart_id}' is shared by several drivers, but a TMC2208 cannot be addressed. "
                "Use one UART bus per TMC2208 or switch to TMC2209 drivers with distinct addresses."
            )
        if address == config[CONF_ADDRESS]:
            raise cv.Invalid(
                f"UART address {address} is used by more than one TMC2209 on uart '{uart_id}'"
            )
    users.append((config[CONF_DRIVER], config[CONF_ADDRESS]))
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


def require_parent(uart_keys=(), tmc2209_keys=(), diag_keys=(), parent_keys=None):
    """Final validator for entity platforms: checks options against the referenced driver."""

    def validator(config):
        full = fv.full_config.get()
        parent_path = full.get_path_for_id(config[CONF_TMC22XX_ID])[:-1]
        parent = full.get_config_for_path(parent_path)
        for key in uart_keys:
            if key in config and parent[CONF_MODE] != MODE_UART:
                raise cv.Invalid(
                    f"'{key}' requires 'mode: uart' on the referenced tmc22xx driver",
                    path=[key],
                )
        for key in tmc2209_keys:
            if key in config and parent[CONF_DRIVER] != DRIVER_TMC2209:
                raise cv.Invalid(
                    f"'{key}' requires 'driver: tmc2209' (the TMC2208 has no StallGuard)",
                    path=[key],
                )
        for key in diag_keys:
            if key in config and CONF_DIAG_PIN not in parent:
                raise cv.Invalid(
                    f"'{key}' requires 'diag_pin' on the referenced tmc22xx driver",
                    path=[key],
                )
        for key, parent_key in (parent_keys or {}).items():
            if key in config and parent_key not in parent:
                raise cv.Invalid(
                    f"'{key}' requires '{parent_key}' on the referenced tmc22xx driver",
                    path=[key],
                )
        return config

    return validator


_CALLBACK_AUTOMATIONS = (
    automation.CallbackAutomation(
        CONF_ON_MOVE_COMPLETE, "add_on_move_complete_callback"
    ),
    automation.CallbackAutomation(
        CONF_ON_HOMING_COMPLETE, "add_on_homing_complete_callback"
    ),
    automation.CallbackAutomation(
        CONF_ON_HOMING_FAILED, "add_on_homing_failed_callback"
    ),
    automation.CallbackAutomation(CONF_ON_STALL, "add_on_stall_callback"),
    automation.CallbackAutomation(
        CONF_ON_DRIVER_ERROR, "add_on_driver_error_callback", [(cg.uint32, "x")]
    ),
)


async def to_code(config):
    # The step generator uses the GPTimer driver, which ESPHome excludes from the build by default.
    include_builtin_idf_component("esp_driver_gptimer")
    # Keep the step ISR alive while the flash cache is disabled (OTA, preference writes).
    add_idf_sdkconfig_option("CONFIG_GPTIMER_ISR_HANDLER_IN_IRAM", True)
    add_idf_sdkconfig_option("CONFIG_GPTIMER_CTRL_FUNC_IN_IRAM", True)
    add_idf_sdkconfig_option("CONFIG_GPTIMER_ISR_CACHE_SAFE", True)

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    cg.add(var.set_driver_type(DRIVER_TYPES[config[CONF_DRIVER]]))
    cg.add(var.set_step_pin(await cg.gpio_pin_expression(config[CONF_STEP_PIN])))
    cg.add(var.set_dir_pin(await cg.gpio_pin_expression(config[CONF_DIR_PIN])))
    if (pin := config.get(CONF_ENABLE_PIN)) is not None:
        cg.add(var.set_enable_pin(await cg.gpio_pin_expression(pin)))
    if (pin := config.get(CONF_MS1_PIN)) is not None:
        cg.add(var.set_ms1_pin(await cg.gpio_pin_expression(pin)))
    if (pin := config.get(CONF_MS2_PIN)) is not None:
        cg.add(var.set_ms2_pin(await cg.gpio_pin_expression(pin)))

    cg.add(var.set_steps_per_unit(config[CONF_STEPS_PER_UNIT]))
    cg.add(var.set_max_speed(config[CONF_MAX_SPEED]))
    cg.add(var.set_acceleration(config[CONF_ACCELERATION]))
    cg.add(
        var.set_deceleration(config.get(CONF_DECELERATION, config[CONF_ACCELERATION]))
    )
    if CONF_MIN_POSITION in config:
        cg.add(var.set_min_position(config[CONF_MIN_POSITION]))
    if CONF_MAX_POSITION in config:
        cg.add(var.set_max_position(config[CONF_MAX_POSITION]))
    cg.add(var.set_invert_direction(config[CONF_INVERT_DIRECTION]))
    cg.add(var.set_enable_on_boot(config[CONF_ENABLE_ON_BOOT]))
    if CONF_IDLE_DISABLE_DELAY in config:
        cg.add(
            var.set_idle_disable_delay(
                config[CONF_IDLE_DISABLE_DELAY].total_milliseconds
            )
        )
    cg.add(var.set_report_interval(config[CONF_REPORT_INTERVAL].total_milliseconds))
    if CONF_MICROSTEPS in config:
        cg.add(var.set_microsteps(config[CONF_MICROSTEPS]))

    if config[CONF_MODE] == MODE_UART:
        cg.add_define("USE_TMC22XX_UART")
        uart_bus = await cg.get_variable(config[CONF_UART_ID])
        cg.add(var.set_uart(uart_bus, config[CONF_ADDRESS]))
        cg.add(var.set_rsense(config[CONF_RSENSE]))
        cg.add(var.set_run_current(config[CONF_RUN_CURRENT] * 1000.0))
        cg.add(var.set_hold_current(config[CONF_HOLD_CURRENT] * 1000.0))
        cg.add(var.set_interpolation(config[CONF_INTERPOLATION]))
        cg.add(var.set_spreadcycle(config[CONF_CHOPPER] == CHOPPER_SPREADCYCLE))
        cg.add(
            var.set_standstill_delay(config[CONF_STANDSTILL_DELAY].total_milliseconds)
        )
        cg.add(var.set_poll_interval(config[CONF_POLL_INTERVAL].total_milliseconds))
        if CONF_STEALTHCHOP_THRESHOLD in config:
            cg.add(var.set_stealthchop_threshold(config[CONF_STEALTHCHOP_THRESHOLD]))
        if config[CONF_DRIVER] == DRIVER_TMC2209:
            cg.add_define("USE_TMC22XX_STALLGUARD")
            if (pin := config.get(CONF_DIAG_PIN)) is not None:
                cg.add(var.set_diag_pin(await cg.gpio_pin_expression(pin)))
            if CONF_STALLGUARD_THRESHOLD in config:
                cg.add(
                    var.set_stallguard_threshold(config[CONF_STALLGUARD_THRESHOLD])
                )
            if CONF_STOP_ON_STALL in config:
                cg.add(var.set_stop_on_stall(config[CONF_STOP_ON_STALL]))

    if (homing := config.get(CONF_HOMING)) is not None:
        cg.add_define("USE_TMC22XX_HOMING")
        cg.add(var.set_homing_source(HOMING_SOURCES[homing[CONF_SOURCE]]))
        if (pin := homing.get(CONF_ENDSTOP_PIN)) is not None:
            cg.add(var.set_homing_endstop_pin(await cg.gpio_pin_expression(pin)))
        cg.add(var.set_homing_positive(homing[CONF_DIRECTION] == "positive"))
        cg.add(var.set_homing_speed(homing[CONF_SPEED]))
        if CONF_SLOW_SPEED in homing:
            cg.add(var.set_homing_slow_speed(homing[CONF_SLOW_SPEED]))
        cg.add(var.set_homing_backoff(homing[CONF_BACKOFF]))
        cg.add(var.set_homing_max_travel(homing[CONF_MAX_TRAVEL]))
        cg.add(var.set_homing_timeout(homing[CONF_TIMEOUT].total_milliseconds))
        cg.add(var.set_homing_debounce(homing[CONF_DEBOUNCE].total_microseconds))
        cg.add(var.set_homing_position(homing[CONF_HOME_POSITION]))
        if CONF_PARK_POSITION in homing:
            cg.add(var.set_homing_park_position(homing[CONF_PARK_POSITION]))
        cg.add(var.set_home_on_boot(homing[CONF_ON_BOOT]))

    await automation.build_callback_automations(var, config, _CALLBACK_AUTOMATIONS)


# ----------------------------------------------------------------------------
# Actions / conditions. They are generated as plain calls on the component
# (register_apply_action), so no C++ action classes are compiled.
# ----------------------------------------------------------------------------

_ID_SCHEMA = cv.Schema({cv.Required(CONF_ID): cv.use_id(TMC22XX)})
_SIMPLE_SCHEMA = automation.maybe_simple_id(_ID_SCHEMA)

automation.register_apply_action(
    "tmc22xx.move_to",
    _ID_SCHEMA.extend({cv.Required(CONF_POSITION): cv.templatable(cv.float_)}),
    automation.ApplyField(CONF_POSITION, "move_to", cg.float_),
)
automation.register_apply_action(
    "tmc22xx.move_by",
    _ID_SCHEMA.extend({cv.Required(CONF_DISTANCE): cv.templatable(cv.float_)}),
    automation.ApplyField(CONF_DISTANCE, "move_by", cg.float_),
)
automation.register_apply_action(
    "tmc22xx.run",
    _ID_SCHEMA.extend(
        {
            cv.Optional(CONF_DIRECTION, default="forward"): cv.templatable(
                _validate_direction
            ),
            cv.Optional(CONF_SPEED): cv.templatable(cv.positive_not_null_float),
        }
    ),
    automation.ApplyField(CONF_SPEED, "set_max_speed", cg.float_),
    automation.ApplyField(CONF_DIRECTION, "run", cg.bool_),
)
automation.register_apply_action(
    "tmc22xx.set_position",
    _ID_SCHEMA.extend({cv.Required(CONF_POSITION): cv.templatable(cv.float_)}),
    automation.ApplyField(CONF_POSITION, "set_position", cg.float_),
)
automation.register_apply_action(
    "tmc22xx.set_speed",
    cv.All(
        _ID_SCHEMA.extend(
            {
                cv.Optional(CONF_MAX_SPEED): cv.templatable(
                    cv.positive_not_null_float
                ),
                cv.Optional(CONF_ACCELERATION): cv.templatable(
                    cv.positive_not_null_float
                ),
                cv.Optional(CONF_DECELERATION): cv.templatable(
                    cv.positive_not_null_float
                ),
            }
        ),
        cv.has_at_least_one_key(CONF_MAX_SPEED, CONF_ACCELERATION, CONF_DECELERATION),
    ),
    automation.ApplyField(CONF_MAX_SPEED, "set_max_speed", cg.float_),
    automation.ApplyField(CONF_ACCELERATION, "set_acceleration", cg.float_),
    automation.ApplyField(CONF_DECELERATION, "set_deceleration", cg.float_),
)
automation.register_apply_action(
    "tmc22xx.set_current",
    cv.All(
        _ID_SCHEMA.extend(
            {
                cv.Optional(CONF_RUN_CURRENT): cv.templatable(_milliamps),
                cv.Optional(CONF_HOLD_CURRENT): cv.templatable(_milliamps),
            }
        ),
        cv.has_at_least_one_key(CONF_RUN_CURRENT, CONF_HOLD_CURRENT),
    ),
    automation.ApplyField(CONF_RUN_CURRENT, "set_run_current", cg.float_),
    automation.ApplyField(CONF_HOLD_CURRENT, "set_hold_current", cg.float_),
)
automation.register_apply_action(
    "tmc22xx.set_microsteps",
    _ID_SCHEMA.extend(
        {
            cv.Required(CONF_MICROSTEPS): cv.templatable(
                cv.one_of(1, 2, 4, 8, 16, 32, 64, 128, 256, int=True)
            )
        }
    ),
    automation.ApplyField(CONF_MICROSTEPS, "set_microsteps", cg.uint16),
)
automation.register_apply_action(
    "tmc22xx.spin_test",
    _ID_SCHEMA.extend({cv.Required(CONF_SPEED): cv.templatable(cv.float_)}),
    automation.ApplyField(CONF_SPEED, "spin_test", cg.float_),
)
automation.register_apply_action(
    "tmc22xx.set_stallguard_threshold",
    _ID_SCHEMA.extend(
        {cv.Required(CONF_THRESHOLD): cv.templatable(cv.int_range(min=0, max=255))}
    ),
    automation.ApplyField(CONF_THRESHOLD, "set_stallguard_threshold", cg.uint8),
)

for _name in ("stop", "emergency_stop", "home", "enable", "disable"):
    automation.register_apply_action(
        f"tmc22xx.{_name}", _SIMPLE_SCHEMA, automation.ApplyCall(f"{_name}()")
    )

automation.register_apply_condition("tmc22xx.is_moving", _SIMPLE_SCHEMA, "is_moving()")
automation.register_apply_condition("tmc22xx.is_homed", _SIMPLE_SCHEMA, "is_homed()")
automation.register_apply_condition("tmc22xx.is_homing", _SIMPLE_SCHEMA, "is_homing()")
