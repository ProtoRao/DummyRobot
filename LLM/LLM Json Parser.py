import ollama, json, sys

# ── Centroid lookup (replace with your YOLO output) ──
def get_centroid(object_name: str) -> list:
    # Sanitize input — normalize to lowercase with underscores
    object_name = object_name.strip().lower().replace(" ", "_")
    
    # Replace with your actual YOLO centroid code
    MOCK_CENTROIDS = {
        "red_cylinder" : [0.3, 0.1, 0.05],
        "green_bin"    : [0.8, 0.4, 0.00],
        "blue_cube"    : [0.2, 0.6, 0.05],
        "yellow_box"   : [0.5, 0.3, 0.10],
    }
    
    coords = MOCK_CENTROIDS.get(object_name)
    if coords is None:
        print(f"  ⚠ Object '{object_name}' not found in scene")
        return None
    
    print(f"  centroid({object_name}) → {coords}")
    return coords

# ── Your robot functions ──────────────────
def move_to(target, waypoint, **kwargs):
    print(f"  → MOVE   : {target} → {waypoint}")
    # ik_solver.move(waypoint)

def grasp(target, **kwargs):
    print(f"  → GRASP  : {target}")
    # gripper.close()

def release(target, **kwargs):
    print(f"  → RELEASE: {target}")
    # gripper.open()

ACTION_MAP = {
    "move_to" : move_to,
    "grasp"   : grasp,
    "release" : release,
}


# ── Stage 1: Extract object names from natural language ──
def extract_objects(user_command: str) -> list:
    """Ask LLM to extract just the object names from the command."""
    
    print("\n[Stage 1] Extracting objects from command...")
    
    response = ollama.chat(
        model="qwen3:4b",
        format="json",
        think=False,
        keep_alive="60m",
        messages=[
            {
                "role": "system",
                "content": """Extract all object names from the command. Output ONLY compact valid JSON. Format: {"objects": ["object1", "object2"]}"""
            },
            {
                "role": "user",
                "content": f'Command: "{user_command}"'
            }
        ]
    )
    
    parsed = json.loads(response['message']['content'])
    objects = parsed.get("objects", [])
    print(f"  Objects found: {objects}")
    return objects


# ── Stage 2: Resolve centroids for all objects ──
def resolve_centroids(objects: list) -> dict:
    print("\n[Stage 2] Resolving centroids...")
    
    coords = {}
    for obj in objects:
        obj_clean = obj.strip().lower().replace(" ", "_")  # sanitize key too
        centroid = get_centroid(obj_clean)
        if centroid is not None:
            coords[obj_clean] = centroid  # store as clean name
    
    print(f"  Resolved: {coords}")
    return coords

# ── Stage 3: Generate motion plan with real coordinates ──
def generate_plan(user_command: str, object_names: list) -> str:
    response = ollama.chat(
        model="qwen3:4b",
        format="json",
        think=False,
        keep_alive="60m",
        messages=[
            {
                "role": "system",
                "content": """/no_think You are a robot motion planner. Output ONLY compact valid JSON.

RULES:
- Always follow this exact sequence: move_to → grasp → move_to → release
- The release target must always be the object being held, not the destination
- Do NOT include waypoints or coordinates — those will be added by the system
- Only output action and target fields

Format: {"steps":[{"action":"move_to","target":"x"},{"action":"grasp","target":"x"},{"action":"move_to","target":"x"},{"action":"release","target":"x"}]}"""
            },
            {
                "role": "user",
                "content": f'Command: "{user_command}"\nAvailable objects: {object_names}'
            }
        ]
    )
    return response['message']['content']

def inject_coordinates(plan: dict, coords: dict) -> dict:
    """Replace/inject real coordinates into move_to steps from centroid lookup."""
    
    for step in plan["steps"]:
        if step["action"] == "move_to":
            target = step["target"]
            if target in coords:
                step["waypoint"] = coords[target]  # injected from get_centroid()
            else:
                print(f"  ⚠ No coordinates found for '{target}'")
                step["waypoint"] = None             # handle missing object
    
    return plan

# ── Stage 4: Execute plan ─────────────────
def validate_plan(plan: dict) -> bool:
    """Check that every grasp is followed by move_to then release."""
    steps = plan.get("steps", [])
    
    for i, step in enumerate(steps):
        if step["action"] == "grasp":
            # next step must be move_to
            if i + 1 >= len(steps) or steps[i+1]["action"] != "move_to":
                print(f"  ⚠ Validation failed: grasp at step {i+1} not followed by move_to")
                return False
            # step after that must be release
            if i + 2 >= len(steps) or steps[i+2]["action"] != "release":
                print(f"  ⚠ Validation failed: no release after move_to at step {i+2}")
                return False
    
    return True


def execute_plan(json_string: str, coords: dict):
    try:
        plan = json.loads(json_string)
    except json.JSONDecodeError as e:
        print(f"  Bad JSON: {e}")
        return

    # Inject real coordinates — LLM never touches them
    plan = inject_coordinates(plan, coords)

    if not validate_plan(plan):
        print("  Plan rejected — invalid step sequence")
        return

    print(f"  Steps: {len(plan['steps'])}\n")
    for i, step in enumerate(plan["steps"]):
        print(f"  Step {i+1}: {step}")
        action = step.pop("action")
        func = ACTION_MAP.get(action)
        if func:
            func(**step)
        else:
            print(f"  ⚠ Unknown action '{action}' — skipping")


def run_pipeline(user_command: str):
    print("=" * 50)
    print(f"Command: \"{user_command}\"")
    print("=" * 50)

    # Stage 1: Extract object names
    objects = extract_objects(user_command)
    if not objects:
        print("No objects found.")
        return

    # Stage 2: Resolve real coordinates from your vision pipeline
    coords = resolve_centroids(objects)
    if not coords:
        print("Could not resolve coordinates.")
        return

    # Stage 3: LLM generates sequence only — no coordinates
    plan_json = generate_plan(user_command, list(coords.keys()))  # pass names only
    print(f"  Plan JSON: {plan_json}")

    # Stage 4: Code injects coordinates, then executes
    execute_plan(plan_json, coords)

    print("\n" + "=" * 50)
    print("Pipeline complete.")
    print("=" * 50)


# ── Warmup ────────────────────────────────
# print("Warming up model...")
# ollama.chat(
#     model="qwen3:4b",
#     format="json",
#     think=False,
#     keep_alive="60m",
#     messages=[{"role": "system", "content": "/no_think"},
#               {"role": "user",   "content": "hi"}]
# )
# print("Model ready.\n")

# ── Run ───────────────────────────────────
run_pipeline(input("Enter prompt: "))

# Try another command — objects automatically resolved
# run_pipeline("move the blue cube on top of the yellow box")