import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_TMC22XX_ID, TMC22XX, require_parent

DEPENDENCIES = ["tmc22xx"]

CONF_DRIVER_STATUS = "driver_status"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TMC22XX_ID): cv.use_id(TMC22XX),
        cv.Required(CONF_DRIVER_STATUS): text_sensor.text_sensor_schema(
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            icon="mdi:chip",
        ),
    }
)

FINAL_VALIDATE_SCHEMA = require_parent(uart_keys=(CONF_DRIVER_STATUS,))


async def to_code(config):
    parent = await cg.get_variable(config[CONF_TMC22XX_ID])
    cg.add_define("USE_TMC22XX_TEXT_SENSOR")
    sens = await text_sensor.new_text_sensor(config[CONF_DRIVER_STATUS])
    cg.add(parent.set_driver_status_text_sensor(sens))
