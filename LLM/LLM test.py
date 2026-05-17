import ollama
import time

system_prompt = """
You are a robot motion planner. Given a natural language command and detected 
object coordinates, output ONLY valid JSON. No explanation, no markdown, no preamble.

Output format:
{
  "steps": [
    {"action": "move_to",  "target": "<object>", "waypoint": [x, y, z]},
    {"action": "grasp",    "target": "<object>"},
    {"action": "move_to",  "target": "<destination>", "waypoint": [x, y, z]},
    {"action": "release",  "target": "<object>"}
  ]
}
"""

user_prompt = """
Command: "pick up the red cylinder, place it in the green bin, then pick up the blue cube and stack it on top of the red cylinder"
Detected objects: {
  "red_cylinder": [0.3, 0.1, 0.05],
  "green_bin":    [0.8, 0.4, 0.00],
  "blue_cube":    [0.2, 0.6, 0.05]
}
"""

# --- Timed inference ---
start = time.time()

response = ollama.chat(
    model="qwen3.5:4b",
    format="json",
    think=False,
    messages=[
        {"role": "system", "content": system_prompt},
        {"role": "user",   "content": user_prompt}
    ]
)

elapsed = time.time() - start

# --- Results ---
content = response['message']['content']
eval_count    = response.get('eval_count', '?')       # tokens generated
eval_duration = response.get('eval_duration', None)   # nanoseconds

tokens_per_sec = (eval_count / (eval_duration / 1e9)) if eval_duration else (eval_count / elapsed)

print(f"\n--- Output ---\n{content}")
print(f"\n--- Performance ---")
print(f"Total time     : {elapsed:.2f}s")
print(f"Tokens out     : {eval_count}")
print(f"Tokens/sec     : {tokens_per_sec:.1f}")