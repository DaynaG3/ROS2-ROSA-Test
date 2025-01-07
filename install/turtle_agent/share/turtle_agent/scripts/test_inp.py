import asyncio

import pyinputplus as pyip
from rich.console import Console


def get_input(prompt: str):
        """Get user input from the console."""
        return pyip.inputStr(prompt, default="help")


async def run():
        console = Console()

        while True:    
            console.print("AAAAAAAAAAA")     
            user_input = get_input("> ")
            # user_input = "help" # ADDED
            
            console.print("Input: " + user_input) # ADDED


def main():
    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        print("ERROR")
    finally:
        print("FINAL ERROR")


if __name__ == "__main__":
    main()