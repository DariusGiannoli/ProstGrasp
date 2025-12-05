from modules.llm import get_llm_response

result = get_llm_response("Pick up the Lego brick and put it in the box.")
print("Reply:", result["reply"])
print("Command:", result["command"])

