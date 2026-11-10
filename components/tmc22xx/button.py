import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv

from . import CONF_HOMING, CONF_TMC22XX_ID, TMC22XX, require_parent, tmc22xx_ns

DEPENDENCIES = ["tmc22xx"]

CONF_HOME = "home"
CONF_STOP = "stop"
CONF_EMERGENCY_STOP = "emergency_stop"

TMC22XXButton = tmc22xx_ns.class_("TMC22XXButton", button.Button)
ButtonFunction = tmc22xx_ns.enum("ButtonFunction")

FUNCTIONS = {
    CONF_HOME: ButtonFunction.BUTTON_HOME,
    CONF_STOP: ButtonFunction.BUTTON_STOP,
    CONF_EMERGENCY_STOP: ButtonFunction.BUTTON_EMERGENCY_STOP,
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(CONF_TMC22XX_ID): cv.use_id(TMC22XX),
            cv.Optional(CONF_HOME): button.button_schema(
                TMC22XXButton, icon="mdi:home-import-outline"
            ),
            cv.Optional(CONF_STOP): button.button_schema(
                TMC22XXButton, icon="mdi:stop"
            ),
            cv.Optional(CONF_EMERGENCY_STOP): button.button_schema(
                TMC22XXButton, icon="mdi:alert-octagon"
            ),
        }
    ),
    cv.has_at_least_one_key(*FUNCTIONS),
)

FINAL_VALIDATE_SCHEMA = require_parent(parent_keys={CONF_HOME: CONF_HOMING})


async def to_code(config):
    parent = await cg.get_variable(config[CONF_TMC22XX_ID])
    cg.add_define("USE_TMC22XX_BUTTON")
    for key, function in FUNCTIONS.items():
        if (conf := config.get(key)) is not None:
            var = await button.new_button(conf)
            await cg.register_parented(var, parent)
            cg.add(var.set_function(function))
