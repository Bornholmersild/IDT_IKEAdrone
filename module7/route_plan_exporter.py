import json

class RoutePlanExporter:
    def __init__(self, filename="mission.plan"):
        self.filename = filename
        self.plan = {
            "fileType": "Plan",
            "geoFence": {
                "polygon": [],
                "version": 1
            },
            "groundStation": "QGroundControl",
            "mission": {
                "cruiseSpeed": 15,
                "firmwareType": 3,
                "hoverSpeed": 5,
                "items": [],
                "plannedHomePosition": [],
                "vehicleType": 2,
                "version": 2
            },
            "rallyPoints": {
                "points": [],
                "version": 1
            },
            "version": 1
        }

    def add_home_position(self, latitude, longitude, altitude):
        self.plan["mission"]["plannedHomePosition"] = [latitude, longitude, altitude]

    def add_waypoint(self, latitude, longitude, altitude, command=16, auto_continue=True, frame=3):
        do_jump_id = len(self.plan["mission"]["items"]) + 1
        waypoint = {
            "autoContinue": auto_continue,
            "command": command,
            "doJumpId": do_jump_id,
            "frame": frame,
            "params": [0, 0, 0, 0, latitude, longitude, altitude],
            "type": "SimpleItem"
        }
        self.plan["mission"]["items"].append(waypoint)

    def export_to_file(self):
        with open(self.filename, "w") as file:
            json.dump(self.plan, file, indent=4, sort_keys=True)
        print(f"Route plan saved to {self.filename}")

