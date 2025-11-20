"""
File: generator_cli.py
Command-line interface for behavior tree generation.

Author: Quentin Dury
"""
import argparse
import logging
from hunav_behavior_tree_generator.core.file_generator import FileGenerator
from hunav_behavior_tree_generator.config.config import setup_logging , SIMULATOR_VERSION
from hunav_behavior_tree_generator.ui.interface_manager import InterfaceManager

def main():
    """
    Main entry point for the CLI tool.
    """
    parser = argparse.ArgumentParser(description='Generate behavior trees for HuNav simulation')
    parser.add_argument('--mode', choices=['scenario', 'manual'], default='scenario',
                       help='Generation mode: scenario (from existing) or manual (from scratch)')
    parser.add_argument('--yaml-file', type=str, default=None,
                       help='Path to a specific YAML scenario file (optional, only for scenario mode)')
    parser.add_argument('--simulator', type=str, default="Gazebo Classic",
                       help='Simulator version to use (optional)')
    
    args = parser.parse_args()
    
    logger = setup_logging()
    ui = InterfaceManager()
    SIMULATOR_VERSION = args.simulator
    logger.info("Starting Behavior Tree Generation CLI")

    try:
        generator = FileGenerator(ui_manager=ui)
        
        if args.mode == 'manual':       # No YAML file used so generate from manual input
            logger.info("Manual mode selected")
            generator.generate_BT_from_manual_input()
            
        else:                           # YAML file used for scenario generation
            logger.info("Scenario mode selected")
            if args.yaml_file:          # YAML option with specific file
                logger.info(f"Using YAML file: {args.yaml_file}")
                generator.generate_BT_from_yaml_file(args.yaml_file)
            else:                       # YAML option
                generator.generate_BT_from_yaml_file()
        
        return 0
        
    except KeyboardInterrupt:
        ui.show_cancellation()
        logger.info("CLI interrupted by user")
        return 1
    except Exception as e:
        ui.show_error(str(e))
        logger.error(f"CLI generation failed: {e}")
        return 1

if __name__ == '__main__':
    exit(main())