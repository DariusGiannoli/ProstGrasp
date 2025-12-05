from modules.executor import execute_command

execute_command({
    "action": "pick_and_place",
    "object": "blue lego brick",
    "from": "floor",
    "to": "box"
})