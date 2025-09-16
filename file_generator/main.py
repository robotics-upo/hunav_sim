import logging
import questionary
from rich.console import Console
from config.settings import load_env_variables
from config.logging_config import setup_logging
from utils.menu_utils import world_generator_menu, behavior_generator_menu

console = Console()


def main():
    """Main entry point for the application."""
    load_env_variables()
    setup_logging()
    logging.info("Program started.")

    while True:
        console.print("\n[bold magenta]✨       Welcome       ✨[/bold magenta]")
        console.print("[cyan]Choose what you want to do:[/cyan]")

        choice = questionary.select(
            "Select an option:",
            choices=[
                "🌍  World Generator",
                "🤖  Behavior Generator",
                "❌  Quit"
            ],
            qmark="👉",
            pointer="➡️"
        ).ask()

        if choice == "🌍  World Generator":
            logging.info("World Generation selected.")
            world_generator_menu()

        elif choice == "🤖  Behavior Generator":
            logging.info("Behavior Generation selected.")
            behavior_generator_menu()

        elif choice == "❌  Quit":
            logging.info("Exiting Program.")
            console.print("[bold magenta]👋 Goodbye![/bold magenta]")
            break


if __name__ == "__main__":
    main()
