#import openai
import re
# from pathlib import Path


from openai import OpenAI

# Use your OpenAI key (starts with 'sk-')
client = OpenAI(api_key ='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')

with open('src/llm_integration/ros_log_latest_run.txt', 'r') as file:
    # Read the entire content of the file into a single string
    log_text = file.read()

# The variable file_contents now holds the entire content of the file as a string
print(log_text)

prompt = f"""You are a quadruped robot tasked with finding purple cups in the environment, testing if they are movable, and then bringing movable cups to the goal.
    Tell a short story from first person perspective of how you navigated the environment based on the provided context.
    The way you test a cup is movable is by crouching down and seeing if it sticks to your velcro beard
    We do not know if the cup is movable until you attempt to pick it up.
    All cups are on the ground, be concise with the story, do not make up details that are not present in the context.
    Your response should be summarizing the story, end the summary with if the mission was a success or not.
    (a success is if the movable cups were delivered to goal and the robot classifies the immovable cups)
    
    context:{log_text} """


print("\n\nGenerating Story...")
completion = client.chat.completions.create(
    model='gpt-4',
    messages = [{
        'role': 'user',
        'content': prompt
    }],
    temperature=0.1,
    top_p=0.1,
    max_tokens=1500,
)

print(completion.choices[0].message.content)

print("\n\nConverting to audio...")
response = client.audio.speech.create(
  model="tts-1",
  voice="echo",
  input=completion.choices[0].message.content
)

response.stream_to_file('src/llm_integration/mission_story.mp3')
print('done')
