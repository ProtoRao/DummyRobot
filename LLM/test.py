import time

import ollama

t0 = time.time()
ollama.chat(
    model="qwen3.5:4b",
    options={"think": False},
    keep_alive="60m",
    messages=[{"role": "user", "content": "hi"}]
)
print(f"Warmup done: {time.time()-t0:.2f}s")

# Add explicit wait to confirm model is fully settled
time.sleep(2)

t1 = time.time()
response = ollama.chat(
    model="qwen3.5:4b",
    format="json",
    options={"think": False, "num_predict": 1},
    keep_alive="60m",
    messages=[{"role": "user", "content": "hi"}]
)
print(f"First token after warmup: {time.time()-t1:.2f}s")
print(f"Prompt tokens: {response.get('prompt_eval_count')}")