import pygame

class Controller:
    def __init__(self, joystick):
        self.joystick = joystick
        self.button_down_handlers = {}
        self.button_up_handlers = {}
        self.axis_handlers = {}

    def on_button_down(self, button, handler):
        self.button_down_handlers[button] = handler

    def on_button_up(self, button, handler):
        self.button_up_handlers[button] = handler

    def on_axis_motion(self, axis, handler):
        self.axis_handlers[axis] = handler

    def handle_event(self, evt):
        if evt.type == pygame.JOYBUTTONDOWN:
            fn = self.button_down_handlers.get(evt.button)
            if fn: fn()
        elif evt.type == pygame.JOYBUTTONUP:
            fn = self.button_up_handlers.get(evt.button)
            if fn: fn()
        elif evt.type == pygame.JOYAXISMOTION:
            fn = self.axis_handlers.get(evt.axis)
            if fn: fn(evt.value)