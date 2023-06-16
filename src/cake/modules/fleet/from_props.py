from .FleetSMRPP import FleetSMRPP

def from_props(props, robot):
    """
    This object becomes available automaically if all the following conditions are met:

    * At least one fleet property with the following keys:
        - fleet_key
        - environment_model
        - robot_id

      Optional Keys include:
        - fleet_server
    """

    fleet = props.get('fleet')
    if fleet is None:
        return None

    if fleet.get('fleet_key') is None:
        return None

    if fleet.get('environment_model') is None:
        return None

    if fleet.get('robot_id') is None:
        return None

    return FleetSMRPP(robot, props)
