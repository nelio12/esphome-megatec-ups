import esphome.codegen as cg
from esphome.components import uart, text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]
CODEOWNERS = ["@syssi"]
AUTO_LOAD = ["binary_sensor", "sensor", "switch", "text_sensor"]
MULTI_CONF = True

CONF_POWERMUST_ID = "powermust_id"
powermust_ns = cg.esphome_ns.namespace("powermust")
PowermustComponent = powermust_ns.class_("Powermust", cg.Component)

POWERMUST_COMPONENT_SCHEMA = cv.Schema({
    cv.Required(CONF_POWERMUST_ID): cv.use_id(PowermustComponent),
})

CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(PowermustComponent),
        cv.Optional("update_interval", default="10s"): cv.update_interval,
        cv.Optional("ups_info"): text_sensor.text_sensor_schema(
            icon="mdi:information-outline"
        ),
    })
    .extend(cv.polling_component_schema("10s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)

# ← ¡CORREGIDO: async def!
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)          # ← await
    await uart.register_uart_device(var, config)      # ← await

    if "ups_info" in config:
        ups_info_conf = config["ups_info"]
        ups_info = await text_sensor.new_text_sensor(ups_info_conf)  # ← await
        cg.add(var.set_ups_info(ups_info))

    yield var  # ← ¡Este yield SÍ va al final!
