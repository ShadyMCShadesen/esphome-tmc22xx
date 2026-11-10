import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import CONF_MAX_VALUE, CONF_MIN_VALUE, CONF_STEP

from . import CONF_TMC22XX_ID, TMC22XX, tmc22xx_ns

DEPENDENCIES = ["tmc22xx"]

TMC22XXTargetNumber = tmc22xx_ns.class_("TMC22XXTargetNumber", number.Number)


def _validate_range(config):
    if config[CONF_MIN_VALUE] >= config[CONF_MAX_VALUE]:
        raise cv.Invalid("min_value must be lower than max_value")
    return config


# The number represents the target position in user units; setting it starts a move.
CONFIG_SCHEMA = cv.All(
    number.number_schema(TMC22XXTargetNumber, icon="mdi:target")
    .extend(
        {
            cv.GenerateID(CONF_TMC22XX_ID): cv.use_id(TMC22XX),
            cv.Required(CONF_MIN_VALUE): cv.float_,
            cv.Required(CONF_MAX_VALUE): cv.float_,
            cv.Optional(CONF_STEP, default=1.0): cv.positive_not_null_float,
        }
    ),
    _validate_range,
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_TMC22XX_ID])
    cg.add_define("USE_TMC22XX_NUMBER")
    var = await number.new_number(
        config,
        min_value=config[CONF_MIN_VALUE],
        max_value=config[CONF_MAX_VALUE],
        step=config[CONF_STEP],
    )
    await cg.register_parented(var, parent)
    cg.add(parent.set_target_number(var))
