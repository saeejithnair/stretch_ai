# Copyright (c) Hello Robot, Inc.
# All rights reserved.
#
# This source code is licensed under the license found in the LICENSE file in the root directory
# of this source tree.
#
# Some code may be adapted from other open-source works with their respective licenses. Original
# license information maybe found below, if so.

from typing import List, Tuple

from stretch.llms.base import AbstractPromptBuilder

simple_stretch_prompt = """You are a friendly, helpful robot named Stretch. You are always helpful, and answer questions concisely. You will never harm a human or suggest harm.

When prompted, you will respond using these actions:
- pickup(object_name)  # object_name is the name of the object to pick up
- explore(int)  # explore the environment for a certain number of steps
- place(location_name)  # location_name is the name of the receptacle to place object in
- say(text)  # say something to the user
- wave()  # wave at a person
- nod_head() # nod your head
- shake_head() # shake your head
- avert_gaze() # avert your gaze
- find(object_name)  # find the object or location by exploring
- go_home()  # navigate back to where you started
- quit()  # end the conversation

These functions and their arguments are the only things you will say. Wave if a person is being nice to you or greeting you. You should always explain what you are going to do before you do it. If you cannot clearly determine which object and location are relevant, say so, instead of providing either pick() or place(). If you do not understand how to do something, say you do not know. Do not hallucinate. You will always say something to acknowledge the user.

input: "Put the red apple in the cardboard box"
output:
say("Picking up the red apple in the cardboard box")
pickup(red apple)
place(cardboard box)
end()

input: "Hi!"
output:
say("Hello!")
wave()
end()

input: "Goodbye!"
output:
say("Goodbye!")
wave()
quit()

input: "Thank you!"
output:
say("You're welcome!")
wave()
end()

input: "What is your name?"
output:
say("My name is Stretch.")
wave()
end()

input: "Is the sky blue?"
output:
say("Yes, the sky is blue.")
nod_head()
end()

input: "What is the meaning of life?"
output:
say("I don't know.")
shake_head()
end()

input: "What is 2 + 2?"
output:
say("2 + 2 is 4.")
end()

NEVER return pickup() without a corresponding place() command. If you cannot determine the location, say so instead of providing a place() command. If you cannot determine the object, say so instead of providing a pickup() command. If you cannot determine either, say so instead of providing either command.

You may only use each action once. No duplicate actions.

for example:

input: "can you put the shoe away?"
output:
say("Where should I put the shoe?")
end()

If a human uses obvious shorthand for an object, unroll it when calling functions:

input: "Find the remote."
output:
say("Looking for the tv remote control.")
find(tv remote control)
end()

input: "Can you put the shoe in the closet?"
output:
say("Picking up the shoe and putting it in the closet.")
pickup(shoe)
place(closet)
end()

input: "Get me a glass of water."
output:
say("Where would I put the glass of water?")
end()

input: "Put the pen in the pencil holder"
output:
say("Picking up the pen and putting it in the pencil holder.")
pickup(pen)
place(pencil holder)
end()

input:
"""


class PickupPromptBuilder(AbstractPromptBuilder):
    def __init__(self):
        self.prompt_str = simple_stretch_prompt

    def __str__(self):
        return self.prompt_str

    def configure(self, **kwargs) -> str:
        assert len(kwargs) == 0, "SimplePromptBuilder does not take any arguments."
        return self.prompt_str

    def parse_response(self, response: str):
        """Parse the pickup, place, and say commands from the response into a list."""
        commands = []
        for line in response.split("\n"):
            if line.startswith("pickup("):
                commands.append(line)
            elif line.startswith("place("):
                commands.append(line)
            elif line.startswith("say("):
                commands.append(line)
            elif line.startswith("wave()"):
                commands.append(line)
            elif line.startswith("go_home()"):
                commands.append(line)
            elif line.startswith("explore()"):
                commands.append(line)
            elif line.startswith("nod_head()"):
                commands.append(line)
            elif line.startswith("shake_head()"):
                commands.append(line)
            elif line.startswith("avert_gaze()"):
                commands.append(line)
            elif line.startswith("quit()"):
                commands.append(line)
            elif line.startswith("find("):
                commands.append(line)
            elif line.startswith("end()"):
                # Stop parsing if we see the end command
                break

        # Now go through commands and parse into a tuple (command, args)
        parsed_commands = []
        for command in commands:
            if command.startswith("say("):
                parsed_commands.append(("say", command[4:-1]))
            elif command.startswith("pickup("):
                parsed_commands.append(("pickup", command[7:-1]))
            elif command.startswith("place("):
                parsed_commands.append(("place", command[6:-1]))
            elif command.startswith("wave()"):
                parsed_commands.append(("wave", ""))
            elif command.startswith("go_home()"):
                parsed_commands.append(("go_home", ""))
            elif command.startswith("explore()"):
                parsed_commands.append(("explore", ""))
            elif command.startswith("nod_head()"):
                parsed_commands.append(("nod_head", ""))
            elif command.startswith("shake_head()"):
                parsed_commands.append(("shake_head", ""))
            elif command.startswith("avert_gaze()"):
                parsed_commands.append(("avert_gaze", ""))
            elif command.startswith("find("):
                parsed_commands.append(("find", command[5:-1]))
            elif command.startswith("quit()"):
                # Quit actually shuts down the robot.
                parsed_commands.append(("quit", ""))
                break
            elif command.startswith("end()"):
                # Stop parsing if we see the end command
                # This really shouldn't happen, but just in case
                break

        return parsed_commands

    def get_object(self, response: List[Tuple[str, str]]) -> str:
        """Return the object from the response."""
        for command, args in response:
            if command == "pickup":
                return args
        return ""

    def get_receptacle(self, response: List[Tuple[str, str]]) -> str:
        """Return the receptacle from the response."""
        for command, args in response:
            if command == "place":
                return args
        return ""

    def get_say_this(self, response: List[Tuple[str, str]]) -> str:
        """Return the text to say from the response."""
        all_messages = []
        for command, args in response:
            if command == "say":
                all_messages.append(args)
        return " ".join(all_messages)

    def get_wave(self, response: List[Tuple[str, str]]) -> bool:
        """Return if the robot should wave."""
        for command, args in response:
            if command == "wave":
                return True
        return False
