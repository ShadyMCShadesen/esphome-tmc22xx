import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_HEAT,
    DEVICE_CLASS_MOVING,
    DEVICE_CLASS_PROBLEM,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

from . import CONF_TMC22XX_ID, TMC22XX, require_parent

DEPENDENCIES = ["tmc22xx"]

CONF_MOVING = "moving"
CONF_HOMED = "homed"
CONF_HOMING = "homing"
CONF_DRIVER_ERROR = "driver_error"
CONF_OVERTEMPERATURE_WARNING = "overtemperature_warning"
CONF_STALL = "stall"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC22XX_ID): cv.use_id(TMC22XX),
        cv.Optional(CONF_MOVING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_MOVING
        ),
        cv.Optional(CONF_HOMED): binary_sensor.binary_sensor_schema(
            icon="mdi:home-map-marker"
        ),
        cv.Optional(CONF_HOMING): binary_sensor.binary_sensor_schema(
            icon="mdi:home-search"
        ),
        cv.Optional(CONF_DRIVER_ERROR): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_OVERTEMPERATURE_WARNING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_HEAT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_STALL): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM
        ),
    }
)

FINAL_VALIDATE_SCHEMA = require_parent(
    uart_keys=(CONF_DRIVER_ERROR, CONF_OVERTEMPERATURE_WARNING, CONF_STALL),
    tmc2209_keys=(CONF_STALL,),
    diag_keys=(CONF_STALL,),
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_TMC22XX_ID])
    cg.add_define("USE_TMC22XX_BINARY_SENSOR")
    for key in (
        CONF_MOVING,
        CONF_HOMED,
        CONF_HOMING,
        CONF_DRIVER_ERROR,
        CONF_OVERTEMPERATURE_WARNING,
        CONF_STALL,
    ):
        if (conf := config.get(key)) is not None:
            sens = await binary_sensor.new_binary_sensor(conf)
            cg.add(getattr(parent, f"set_{key}_binary_sensor")(sens))
