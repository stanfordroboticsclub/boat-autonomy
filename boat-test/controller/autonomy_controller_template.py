from controller.base_controller import BaseController
from boat_simulation.simple import Action


# Copy this class and modify the choose_action() method to create a new autonomy controller.
class AutonomyControllerTemplate(BaseController):
    def __init__(self):
        BaseController.__init__(self, "Empty Autonomy Controller")

    def select_action_from_state(self, env, state):
        return Action(0, 0)
