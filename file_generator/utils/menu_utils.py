import logging
import questionary
import shutil
from rich.console import Console
from core.generator import generate_world_from_prompt
from core.adjustor import adjust_world_file
from core.behavior import bt_generation
from utils.file_utils import file_exist

console = Console()

def world_generator_menu():
    """
    Handle the World Generator submenu.
    """
    while True:
        console.print("\n[bold cyan]🌍 World Generator[/bold cyan]")
        console.print("[green]Choose an action below:[/green]")

        choice = questionary.select(
            "Choose an option:",
            choices=[
                "🆕 Generate a new world",
                "⚙️  Adjust the generated world",
                "💾 Save the generated world",
                "⬅️  Back",
                "❌ Quit"
            ],
            qmark="👉",
            pointer="➡️"
        ).ask()

        if choice == "🆕 Generate a new world":
            logging.info("World generator started.")
            generate_world_loop()

        elif choice == "⚙️  Adjust the generated world":
            logging.info("World adjustment started.")
            adjust_world_loop()

        elif choice == "💾 Save the generated world":
            logging.info("World saving started.")
            save_world_loop()

        elif choice == "⬅️  Back":
            logging.info("Back to Main Menu.")
            break

        elif choice == "❌ Quit":
            logging.info("Exiting Program.")
            exit(0)

def generate_world_loop():
    """
    Loop for generating new worlds.
    """
    while True:
        user_prompt = input(
            "✏️ Describe the environment (or type 'b' or 'back' to return to menu): "
        ).strip()

        if not user_prompt:
            console.print("[yellow]⚠️ Please enter a description.[/yellow]")
            continue
        if user_prompt.lower() in ["back", "b"]:
            logging.info("Back to world menu.")
            break
        if user_prompt.lower() in ["exit", "quit", "q"]:
            logging.info("Exiting World Generator.")
            exit(0)

        try:
            sdf_path = generate_world_from_prompt(user_prompt)
            logging.info(f"World file generated at: {sdf_path}")
            console.print(f"[green]✅ World generated → {sdf_path}[/green]")

            logging.info("Back to World Generation Menu")
            console.print(f"[yellow]⬅️  Back to previous menu[/yellow]")
            break

        except Exception as error:
            logging.error(f"Error during world generation: {error}")
            console.print(f"[red]❌ Error: {error}[/red]")
            

def adjust_world_loop():
    """
    Loop for adjusting the generated world.
    """
    while True:
        instruction = input(
            "🛠️ Enter adjustment instruction (or type 'b' or 'back' to return to menu): "
        ).strip()
        
        if instruction.lower() in ["back", "b"]:
            logging.info("Back to world menu.")
            break
        if instruction.lower() in ["exit", "quit", "q"]:
            logging.info("Exiting World Generator.")
            exit(0)

        try:
            world_path = "data/generated_world.sdf"
            adjust_world_file(instruction, world_path)
            logging.info(f"World file adjusted at: {world_path}")
            console.print(f"[green]✅ Adjustment applied to {world_path}[/green]")

            logging.info("Back to World Generation Menu")
            console.print(f"[yellow]⬅️  Back to previous menu[/yellow]")
            break

        except Exception as error:
            logging.error(f"Error during world adjustment: {error}")
            console.print(f"[red]❌ Error: {error}[/red]")

def save_world_loop():
    while True:
        user_input = input(
            "💾 Enter a name for the world file (or type 'b' or 'back' to return to menu): "
        ).strip()

        if user_input.lower() in ["back", "b"]:
            logging.info("Back to world menu.")
            break
        if user_input.lower() in ["/exit", "exit", "quit", "q"]:
            logging.info("Exiting World Generator.")
            exit(0)

        try:
            sdf_path = f"data/saved_worlds/{user_input}.sdf"
            if file_exist(sdf_path):
                console.print(f"[red]❌ Error: File {sdf_path} already exists! Choose another name.[/red]")
                logging.warning(f"File {sdf_path} already exists! Choose another name.")
            else:
                shutil.copy("data/generated_world.sdf", sdf_path)
                logging.info(f"World file saved at: {sdf_path}")
                console.print(f"[green]✅ World saved → {sdf_path}[/green]")
                logging.info("Back to World Generation Menu")
                console.print(f"[yellow]⬅️  Back to previous menu[/yellow]")
                break
        except Exception as error:
            logging.error(f"Error during world saving: {error}")
            console.print(f"[red]❌ Error: {error}[/red]")


def behavior_generator_menu():
    """
    Handle the Behavior Generator submenu.
    """
    while True:
        console.print("\n[bold cyan]🤖 Behavior Generator[/bold cyan]")
        console.print("[green]Choose an action below:[/green]")

        choice = questionary.select(
            "Choose an option:",
            choices=[
                "🆕 Generate a new BT",
                "💾 Save the generated BT",
                "⬅️  Back",
                "❌ Quit"
            ],
            qmark="👉",
            pointer="➡️"
        ).ask()


        if choice == "🆕 Generate a new BT":
            logging.info("Behavior tree generator started.")
            generate_BT_loop()

        elif choice == "💾 Save the generated BT":
            logging.info("Behavior tree saving started.")
            save_BT_loop()

        elif choice == "⬅️  Back":
            logging.info("Back to Main Menu.")
            break

        elif choice == "❌ Quit":
            logging.info("Exiting Program.")
            exit(0)


def generate_BT_loop():
    while True:
        user_input = input(
            "🤖 Describe the behavior tree you want (or type 'b' or 'back' to return to menu): "
        ).strip()

        if user_input.lower() in ["back", "b"]:
            logging.info("Back to world menu.")
            break
        if user_input.lower() in ["/exit", "exit", "quit", "q"]:
            logging.info("Exiting Behavior Generator.")
            exit(0)

        try:
            bt_generation(user_input)
            logging.info("Behavior tree generated successfully!")
            console.print("[green]✅ Behavior tree generated successfully![/green]")

            logging.info("Back to Behavior Generation Menu")
            console.print(f"[yellow]⬅️  Back to previous menu[/yellow]")
            break

        except Exception as error:
            logging.error(f"Error during behavior generation: {error}")
            console.print(f"[red]❌ Error: {e}[/red]")


def save_BT_loop():
    while True:
        user_input = input(
            "💾 Enter a name for the behavior tree file (or type 'b' or 'back' to return to menu): "
        ).strip()

        if user_input.lower() in ["back", "b"]:
            logging.info("Back to behavior menu.")
            break
        if user_input.lower() in ["/exit", "exit", "quit", "q"]:
            logging.info("Exiting Behavior Generator.")
            exit(0)

        try:
            bt_path = f"data/saved_BT/{user_input}.xml"
            if file_exist(bt_path):
                console.print(f"[red]❌ Error: File {bt_path} already exists! Choose another name.[/red]")
                logging.warning(f"File {bt_path} already exists! Choose another name.")
            else :
                shutil.copy("data/generated_BT.xml", bt_path)
                logging.info(f"Behavior tree file saved at: {bt_path}")
                console.print(f"[green]✅ Behavior tree saved → {bt_path}[/green]")
                logging.info("Back to Behavior Generation Menu")
                console.print(f"[yellow]⬅️  Back to previous menu[/yellow]")
                break
        except Exception as error:
            logging.error(f"Error during behavior tree saving: {error}")
            console.print(f"[red]❌ Error: {error}[/red]")


