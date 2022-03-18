add_executable(controller
        controller.c
)

# pull in common dependencies
target_link_libraries(controller
        pico_stdlib
        hardware_i2c
        hardware_pwm
        hardware_irq
        # freertos
)

pico_sdk_init()

# create map/bin/hex file etc.
pico_add_extra_outputs(controller)

# add url via pico_set_program_url
example_auto_set_url(controller)

pico_enable_stdio_usb(controller 1)
pico_enable_stdio_uart(controller 0)