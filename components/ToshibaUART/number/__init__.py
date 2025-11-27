import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    UNIT_CELSIUS,
)
from .. import ToshibaUART, CONF_TOSHIBAUART_ID

CONF_ZONE1_TARGET_TEMP = "zone1_target_temp"
CONF_HOTWATER_TARGET_TEMP = "hotwater_target_temp"

ICON_THERMOMETER = "mdi:thermometer"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_TOSHIBAUART_ID): cv.use_id(ToshibaUART),
        cv.Optional(CONF_ZONE1_TARGET_TEMP): number.number_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            device_class=DEVICE_CLASS_TEMPERATURE,
            # Zone 1 can do cooling (7-30°C) or heating (22-55°C)
            # Set range to accommodate both modes
            min_value=7,
            max_value=55,
            step=1,
        ),
        cv.Optional(CONF_HOTWATER_TARGET_TEMP): number.number_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            device_class=DEVICE_CLASS_TEMPERATURE,
            # Hotwater range
            min_value=40,
            max_value=65,
            step=1,
        ),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_TOSHIBAUART_ID])

    if zone1_config := config.get(CONF_ZONE1_TARGET_TEMP):
        num = await number.new_number(
            zone1_config,
            min_value=7,
            max_value=55,
            step=1,
        )
        cg.add(hub.set_zone1_target_temp_number(num))
        cg.add(num.set_parent(hub))

    if hotwater_config := config.get(CONF_HOTWATER_TARGET_TEMP):
        num = await number.new_number(
            hotwater_config,
            min_value=40,
            max_value=65,
            step=1,
        )
        cg.add(hub.set_hotwater_target_temp_number(num))
        cg.add(num.set_parent(hub))
