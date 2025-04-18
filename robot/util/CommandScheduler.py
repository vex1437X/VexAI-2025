class CommandScheduler:
    def __init__(self):
        self.active_commands = []

    def schedule(self, command):
        command.start()
        self.active_commands.append(command)

    def run(self):
        for command in list(self.active_commands):
            command.execute()
            if command.is_finished():
                command.end(False)
                self.active_commands.remove(command)