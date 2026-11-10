import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_POSITION,
    DEVICE_CLASS_CURRENT,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_MILLIAMP,
)

from . import CONF_TMC22XX_ID, TMC22XX, require_parent

DEPENDENCIES = ["tmc22xx"]

CONF_STALLGUARD_RESULT = "stallguard_result"
CONF_ACTUAL_CURRENT = "actual_current"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC22XX_ID): cv.use_id(TMC22XX),
        cv.Optional(CONF_POSITION): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:axis-arrow",
        ),
        cv.Optional(CONF_STALLGUARD_RESULT): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:gauge",
        ),
        cv.Optional(CONF_ACTUAL_CURRENT): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIAMP,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
    }
)

FINAL_VALIDATE_SCHEMA = require_parent(
    uart_keys=(CONF_STALLGUARD_RESULT, CONF_ACTUAL_CURRENT),
    tmc2209_keys=(CONF_STALLGUARD_RESULT,),
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_TMC22XX_ID])
    cg.add_define("USE_TMC22XX_SENSOR")
    for key in (CONF_POSITION, CONF_STALLGUARD_RESULT, CONF_ACTUAL_CURRENT):
        if (conf := config.get(key)) is not None:
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(parent, f"set_{key}_sensor")(sens))
